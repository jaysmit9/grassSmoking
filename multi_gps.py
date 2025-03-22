#!/usr/bin/env python3

import socket
import time
import json
import argparse
from datetime import datetime
import sys
import os

class GpsdClient:
    """GPSD client for monitoring multiple GPS devices"""
    
    def __init__(self, host='localhost', port=2947, debug=False):
        self.host = host
        self.port = port
        self.socket = None
        self.debug = debug
        self.devices = {}  # Store info about each device
        self.last_data_time = datetime.now()
        self.connect()
        
    def log(self, message):
        if self.debug:
            print(f"[DEBUG] {message}")
            
    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            
            # First check devices
            self.socket.send(b'?DEVICES;\n')
            time.sleep(0.5)
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"DEVICES response: {data}")
            
            try:
                # Parse device list to see what's available
                for line in data.split('\n'):
                    if not line.strip():
                        continue
                    report = json.loads(line)
                    if report.get('class') == 'DEVICES':
                        devices = report.get('devices', [])
                        self.log(f"Found {len(devices)} devices:")
                        for device in devices:
                            self.log(f"  Path: {device.get('path')}, Driver: {device.get('driver')}")
            except json.JSONDecodeError as e:
                self.log(f"JSON parse error for DEVICES: {e}")
            
            # Enable watching with device field reporting
            watch_command = b'?WATCH={"enable":true,"json":true,"device":true,"nmea":true,"raw":1};\n'
            self.socket.send(watch_command)
            self.log(f"Sent WATCH command: {watch_command.decode()}")
            time.sleep(0.5)
            
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"WATCH response: {data}")
            
            return True
        except Exception as e:
            print(f"Error connecting: {e}")
            return False
            
    def process_reports(self, timeout=5.0):
        """Process all available reports from GPSD"""
        if not self.socket:
            return None
            
        try:
            # Use a shorter timeout to process data chunks more efficiently
            self.socket.settimeout(timeout)
            data = self.socket.recv(8192).decode('utf-8').strip()
            
            if not data:
                return None
                
            self.log(f"Received data length: {len(data)} bytes")
            
            # Process each line as a separate JSON object
            reports = []
            for line in data.split('\n'):
                if not line.strip():
                    continue
                try:
                    report = json.loads(line)
                    reports.append(report)
                    
                    # Track devices based on their path
                    if report.get('class') == 'TPV' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Get basic position and fix information
                        status = report.get('status', 0)
                        mode = report.get('mode', 0)
                        
                        # Determine the complete fix mode with detailed information
                        fix_mode = self.determine_fix_mode(report, mode, status)
                        
                        # Store device info
                        self.devices[device_path] = {
                            'last_update': datetime.now(),
                            'lat': report.get('lat', 0.0),
                            'lon': report.get('lon', 0.0),
                            'alt': report.get('alt', 0.0),
                            'track': report.get('track', 0.0),
                            'speed': report.get('speed', 0.0),
                            'mode': mode,
                            'status': status,
                            'fix_mode': fix_mode,
                            'eph': report.get('eph', 0.0),  # Horizontal position error
                            'epv': report.get('epv', 0.0),  # Vertical position error
                            'sep': report.get('sep', 0.0),  # Estimated position error
                            'raw_data': report.get('raw', '')  # Store raw NMEA for analysis
                        }
                    
                    # Process SKY reports for satellite info
                    elif report.get('class') == 'SKY' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Update satellite info if device exists
                        if device_path in self.devices:
                            self.devices[device_path].update({
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            })
                        else:
                            # Create new device entry if it doesn't exist
                            self.devices[device_path] = {
                                'last_update': datetime.now(),
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            }
                            
                    # Check for raw NMEA data with RTK info
                    elif report.get('class') == 'NMEA' and 'device' in report:
                        device_path = report.get('device')
                        sentence = report.get('string', '')
                        
                        # Look for GGA sentence which contains RTK info
                        if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                            parts = sentence.split(',')
                            if len(parts) >= 7:
                                quality = parts[6]
                                if quality in ['4', '5']:  # RTK fix or float
                                    if device_path in self.devices:
                                        fix_mode = "RTK_FIX" if quality == '4' else "RTK_FLOAT"
                                        self.devices[device_path]['fix_mode'] = fix_mode
                                        self.devices[device_path]['rtk_capable'] = True
                                        self.log(f"RTK mode detected from NMEA for {device_path}: {quality} → {fix_mode}")
                        
                except json.JSONDecodeError as e:
                    self.log(f"JSON error: {e} for line: {line[:50]}...")
                    
            return reports
        except socket.timeout:
            self.log("Socket timeout")
            return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def determine_fix_mode(self, report, mode, status):
        """Determine the detailed fix mode from various indicators"""
        
        # Start with basic mode mapping
        mode_map = {0: 'NO DATA', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX'}
        fix_mode = mode_map.get(mode, 'UNKNOWN')
        
        # Enhanced mode detection based on status
        status_map = {
            0: None,           # Unknown
            1: None,           # Normal, use basic mode
            2: "DGPS",         # Differential GPS
            3: "RTK_FIX",      # RTK Fixed solution
            4: "RTK_FLOAT",    # RTK Float solution
            5: "DR",           # Dead Reckoning
            6: "GNSS+DR",      # Combined GNSS and DR
            7: "TIME_ONLY",    # Time only
            8: "SIM",          # Simulation
            9: "SBAS"          # WAAS/EGNOS/MSAS
        }
        
        if status in status_map and status_map[status]:
            fix_mode = status_map[status]
        
        # Check for RTCM data, which indicates RTK corrections
        if 'rtcm' in report:
            self.log(f"RTCM data found in report")
            if fix_mode == '3D FIX':  # Only upgrade mode if already have basic 3D fix
                fix_mode = "RTK_DATA"  # Receiving RTK data but not yet applied
        
        # Check GGA sentence for RTK info (most reliable)
        if 'raw' in report:
            raw_data = report.get('raw', '')
            if '$GNGGA' in raw_data or '$GPGGA' in raw_data:
                gga_parts = raw_data.split(',')
                if len(gga_parts) >= 7:
                    quality = gga_parts[6]
                    quality_map = {
                        '0': 'NO FIX',        # Invalid
                        '1': 'GPS',           # GPS SPS
                        '2': 'DGPS',          # DGPS
                        '3': 'PPS',           # PPS
                        '4': 'RTK_FIX',       # RTK Fixed
                        '5': 'RTK_FLOAT',     # RTK Float
                        '6': 'ESTIMATED',     # Estimated/DR
                        '7': 'MANUAL',        # Manual
                        '8': 'SIMULATION',    # Simulation
                        '9': 'WAAS'           # WAAS/SBAS
                    }
                    if quality in quality_map:
                        fix_mode = quality_map[quality]
                        self.log(f"Mode from GGA quality: {quality} → {fix_mode}")
        
        return fix_mode
    
    def get_device_status(self):
        """Return status of all devices"""
        return self.devices

    def determine_device_roles(self):
        """Identify which device is rover and which is base"""
        rover = None
        base = None
        unknown_devices = []
        
        for device_path, data in self.devices.items():
            # A device is likely a rover if it:
            # 1. Shows RTK_FIX or RTK_FLOAT mode
            # 2. Has RTCM data
            # 3. Has much higher precision (lower eph)
            if (data.get('fix_mode') in ['RTK_FIX', 'RTK_FLOAT'] or 
                data.get('rtk_capable', False) or
                (data.get('eph', 99) < 0.5 and data.get('mode') == 3)):
                
                rover = device_path
                self.log(f"Identified rover: {device_path}")
                
            # Other devices with 3D fix are likely base stations
            elif data.get('mode') == 3:
                base = device_path
                self.log(f"Identified base: {device_path}")
            else:
                unknown_devices.append(device_path)
        
        return {
            'rover': rover,
            'base': base,
            'unknown': unknown_devices
        }

    def send_poll(self):
        """Send poll command to gpsd"""
        if self.socket:
            try:
                self.socket.send(b'?POLL;\n')
                return True
            except:
                return False
        return False

def format_gps_data(device_path, data, role=None):
    """Format GPS data for display with comprehensive mode information"""
    # Determine device role label
    device_role = role if role else os.path.basename(device_path)
    if role == 'rover':
        device_role = "ROVER (FRONT)"
    elif role == 'base':
        device_role = "BASE (REAR)"
    
    # Create header
    output = [f"GPS: {device_role} ({device_path})"]
    
    # Position data
    if 'lat' in data and 'lon' in data:
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        lat_dir = 'N' if lat >= 0 else 'S'
        lon_dir = 'E' if lon >= 0 else 'W'
        
        output.extend([
            f"  Location: {abs(lat):.7f}°{lat_dir}, {abs(lon):.7f}°{lon_dir}",
            f"  Altitude: {data.get('alt', 0.0):.1f}m",
            f"  Heading: {data.get('track', 0.0):.1f}°",
            f"  Speed: {data.get('speed', 0.0) * 3.6:.1f}km/h"
        ])
        
    # Add precision data
    if 'eph' in data or 'epv' in data or 'sep' in data:
        precision_info = []
        if 'eph' in data:
            precision_info.append(f"H±{data.get('eph', 0.0):.2f}m")
        if 'epv' in data:
            precision_info.append(f"V±{data.get('epv', 0.0):.2f}m")
        if 'sep' in data:
            precision_info.append(f"3D±{data.get('sep', 0.0):.2f}m")
        
        output.append(f"  Precision: {', '.join(precision_info)}")
    
    # Fix mode and quality
    fix_mode = data.get('fix_mode', 'UNKNOWN')
    output.append(f"  Mode: {fix_mode}")
    
    # Satellite data
    if 'satellites_used' in data:
        output.append(f"  Satellites: {data.get('satellites_used', 0)}/{data.get('satellites_visible', 0)}")
        
    if 'hdop' in data:
        output.append(f"  DOP: H:{data.get('hdop', 0.0):.2f} V:{data.get('vdop', 0.0):.2f} P:{data.get('pdop', 0.0):.2f}")
    
    # Timestamp
    if 'last_update' in data:
        output.append(f"  Last update: {data.get('last_update').strftime('%H:%M:%S')}")
    
    return "\n".join(output)

def main():
    parser = argparse.ArgumentParser(description="Monitor multiple GPS devices with comprehensive mode detection")
    parser.add_argument("-d", "--debug", action="store_true", help="Enable debug output")
    parser.add_argument("-i", "--interval", type=float, default=1.0, help="Update interval")
    parser.add_argument("-n", "--no-clear", action="store_true", help="Don't clear screen between updates")
    args = parser.parse_args()
    
    client = GpsdClient(debug=args.debug)
    
    try:
        print("Monitoring multiple GPS devices. Press Ctrl+C to exit.")
        print("Waiting for GPS data...")
        
        poll_count = 0
        
        while True:
            # Poll every few cycles for data
            if poll_count % 10 == 0:
                client.send_poll()
            poll_count += 1
            
            # Process reports
            client.process_reports()
            
            # Display device status
            devices = client.get_device_status()
            
            if devices:
                # Determine device roles (rover vs base)
                roles = client.determine_device_roles()
                
                if not args.no_clear:
                    os.system('cls' if os.name == 'nt' else 'clear')
                
                print(f"Connected to {len(devices)} GPS device(s) at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                print("-" * 60)
                
                # Display rover first
                if roles['rover'] and roles['rover'] in devices:
                    print(format_gps_data(roles['rover'], devices[roles['rover']], 'rover'))
                    print("-" * 60)
                
                # Then base
                if roles['base'] and roles['base'] in devices:
                    print(format_gps_data(roles['base'], devices[roles['base']], 'base'))
                    print("-" * 60)
                
                # Then any unknown devices
                for device_path in devices:
                    if (not roles['rover'] or device_path != roles['rover']) and (not roles['base'] or device_path != roles['base']):
                        print(format_gps_data(device_path, devices[device_path]))
                        print("-" * 60)
                        
            else:
                print("Waiting for GPS data...", end="\r")
                
            time.sleep(args.interval)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if hasattr(client, 'socket') and client.socket:
            client.socket.close()

if __name__ == "__main__":
    main()