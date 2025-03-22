#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/hardware/service_gps.py

import serial
import os
import time
import math
import sys
import glob
from datetime import datetime
import threading

# ANSI color codes for prettier output
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_MAGENTA = "\033[35m"
COLOR_CYAN = "\033[36m"
COLOR_RED = "\033[31m"

class DirectGpsMonitor:
    """GPS Monitor that directly reads from serial ports"""
    
    def __init__(self):
        self.devices = {}  # Store device info by port name
        self.threads = {}  # Reader threads
        self.running = True
        self.lock = threading.Lock()  # For thread-safe device dictionary updates
    
    def find_serial_ports(self):
        """Find available serial ports"""
        if sys.platform.startswith('linux'):
            return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        elif sys.platform.startswith('win'):
            # Windows ports
            return ['COM%s' % (i + 1) for i in range(256)]
        else:
            return glob.glob('/dev/tty.*') + glob.glob('/dev/cu.*')
    
    def add_device(self, port, baud_rate=115200):
        """Add a GPS device to monitor"""
        # Create thread-safe device entry
        with self.lock:
            self.devices[port] = {
                'port': port,
                'baud_rate': baud_rate,
                'connected': False,
                'last_update': time.time(),
                'rtk_mode': None,
                'gga_quality': 0,
                'lat': None,
                'lon': None,
                'alt': None,
                'speed': None,
                'track': None,
                'satellites_used': 0,
                'satellites_visible': 0,
                'hdop': 0.0
            }
        
        # Start monitoring thread
        thread = threading.Thread(
            target=self._monitor_thread,
            args=(port, baud_rate),
            daemon=True
        )
        self.threads[port] = thread
        thread.start()
        
        print(f"Added GPS device: {port} @ {baud_rate} baud")
        return True
    
    def _monitor_thread(self, port, baud_rate):
        """Thread to monitor a single GPS device"""
        ser = None
        buffer = ""
        sentence_counts = {}
        
        try:
            ser = serial.Serial(port, baud_rate, timeout=0.1)
            
            # Update connected status
            with self.lock:
                self.devices[port]['connected'] = True
            
            while self.running:
                try:
                    # Read from serial port
                    data = ser.read(1024)
                    if data:
                        # Decode and add to buffer
                        text = data.decode('ascii', errors='replace')
                        buffer += text
                        
                        # Process complete sentences
                        while '\r\n' in buffer:
                            sentence, buffer = buffer.split('\r\n', 1)
                            
                            # Only process valid NMEA sentences
                            if sentence.startswith('$'):
                                # Update counts
                                sentence_type = sentence[:6]
                                
                                with self.lock:
                                    if 'sentences' not in self.devices[port]:
                                        self.devices[port]['sentences'] = {}
                                    
                                    self.devices[port]['sentences'][sentence_type] = \
                                        self.devices[port]['sentences'].get(sentence_type, 0) + 1
                                    self.devices[port]['last_update'] = time.time()
                                
                                # Process specific sentence types
                                self._process_nmea_sentence(port, sentence)
                    
                    time.sleep(0.01)  # Short delay to prevent CPU hogging
                    
                except Exception as e:
                    print(f"Error reading from {port}: {e}")
                    time.sleep(1)  # Wait before retry
        
        except serial.SerialException as e:
            print(f"Error connecting to {port}: {e}")
        
        finally:
            if ser and ser.is_open:
                ser.close()
            
            # Update connection status
            with self.lock:
                if port in self.devices:
                    self.devices[port]['connected'] = False
    
    def _process_nmea_sentence(self, port, sentence):
        """Process NMEA sentence to extract GPS data"""
        parts = sentence.split(',')
        sentence_type = sentence[:6]
        
        try:
            # GGA - Global Positioning System Fix Data
            if sentence_type in ['$GPGGA', '$GNGGA']:
                if len(parts) >= 15:
                    with self.lock:
                        # Time
                        time_str = parts[1]
                        
                        # Position
                        if parts[2] and parts[4]:  # lat & lon are not empty
                            try:
                                # Convert from DDMM.MMMM to decimal degrees
                                lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
                                if parts[3] == 'S':
                                    lat = -lat
                                
                                lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
                                if parts[5] == 'W':
                                    lon = -lon
                                    
                                self.devices[port]['lat'] = lat
                                self.devices[port]['lon'] = lon
                            except ValueError:
                                pass
                            
                        # Fix quality
                        if parts[6]:
                            quality = int(parts[6])
                            self.devices[port]['gga_quality'] = quality
                            
                            # Map quality to RTK mode
                            if quality == 4:
                                self.devices[port]['rtk_mode'] = 'RTK_FIX'
                            elif quality == 5:
                                self.devices[port]['rtk_mode'] = 'RTK_FLOAT'
                            elif quality == 2:
                                self.devices[port]['rtk_mode'] = 'DGPS'
                            elif quality == 1:
                                self.devices[port]['rtk_mode'] = '3D_FIX'
                            else:
                                self.devices[port]['rtk_mode'] = 'NO_FIX'
                        
                        # Satellites used
                        if parts[7]:
                            self.devices[port]['satellites_used'] = int(parts[7])
                        
                        # HDOP
                        if parts[8]:
                            self.devices[port]['hdop'] = float(parts[8])
                        
                        # Altitude
                        if parts[9]:
                            self.devices[port]['alt'] = float(parts[9])
            
            # RMC - Recommended Minimum Navigation Information
            elif sentence_type in ['$GPRMC', '$GNRMC']:
                if len(parts) >= 12:
                    with self.lock:
                        # Speed in knots converted to m/s
                        if parts[7]:
                            try:
                                speed_knots = float(parts[7])
                                self.devices[port]['speed'] = speed_knots * 0.514444  # knots to m/s
                            except ValueError:
                                pass
                        
                        # Track/Course over ground
                        if parts[8]:
                            try:
                                self.devices[port]['track'] = float(parts[8])
                            except ValueError:
                                pass
            
            # GSV - Satellites in View
            elif sentence_type in ['$GPGSV', '$GNGSV']:
                if len(parts) >= 4:
                    with self.lock:
                        # Total number of satellites in view
                        if parts[3]:
                            try:
                                self.devices[port]['satellites_visible'] = int(parts[3])
                            except ValueError:
                                pass
        
        except Exception as e:
            print(f"Error processing NMEA sentence: {e}")
    
    def get_fix_mode_display(self, data):
        """Return colored status indicator based on device data"""
        # Check for RTK status
        rtk_mode = data.get('rtk_mode')
        quality = data.get('gga_quality', 0)
        
        # Build status text
        if rtk_mode == 'RTK_FIX':
            return f"[RTK FIXED] 🟢"
        elif rtk_mode == 'RTK_FLOAT':
            return f"[RTK FLOAT] 🟡"
        elif rtk_mode == 'DGPS':
            return f"[DGPS] 🔵"
        elif rtk_mode == '3D_FIX':
            return f"[3D FIX] 🔵"
        elif rtk_mode == 'NO_FIX':
            return f"[NO FIX] 🔴"
        elif quality > 0:
            return f"[FIX QUALITY: {quality}] ⚪"
        else:
            return f"[NO DATA] ⚫"
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from point1 to point2"""
        try:
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
            
            bearing = math.atan2(y, x)
            bearing = math.degrees(bearing)
            bearing = (bearing + 360) % 360
            
            return bearing
        except Exception as e:
            print(f"Error calculating bearing: {e}")
            return None
    
    def close(self):
        """Clean shutdown of monitor and threads"""
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads.values():
            thread.join(timeout=1.0)
        
        print("GPS Monitor closed")

def test_baud_rate(port, baud_rate):
    """Test if a port works with a specific baud rate"""
    try:
        with serial.Serial(port, baud_rate, timeout=1.0) as ser:
            # Wait a moment for connection to establish
            time.sleep(0.2)
            
            # Try to read some data
            start_time = time.time()
            while time.time() - start_time < 2.0:  # Try for 2 seconds
                data = ser.read(256)
                if data:
                    # Look for GPS NMEA sentences
                    text = data.decode('ascii', errors='replace')
                    if '$GP' in text or '$GN' in text:
                        return True
        return False
    except serial.SerialException:
        return False

def find_working_baud_rate(port):
    """Find working baud rate for a GPS device"""
    # Common baud rates for GPS devices
    baud_rates = [115200, 9600, 38400, 4800]
    
    print(f"Testing baud rates for {port}...")
    
    for baud in baud_rates:
        print(f"  Trying {baud} baud... ", end='', flush=True)
        if test_baud_rate(port, baud):
            print("SUCCESS!")
            return baud
        print("failed")
    
    print(f"No working baud rate found for {port}")
    return None

def main():
    """Main function"""
    monitor = DirectGpsMonitor()
    
    # Find available ports
    ports = monitor.find_serial_ports()
    
    if not ports:
        print("No serial ports found!")
        return
    
    print("Available GPS ports:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port}")
    
    print("\nSelect GPS ports to monitor (comma-separated numbers, or 'all'):")
    choice = input("> ")
    
    selected_ports = []
    
    if choice.lower() == 'all':
        selected_ports = ports
    else:
        try:
            for idx in choice.split(','):
                port_idx = int(idx.strip()) - 1
                if 0 <= port_idx < len(ports):
                    selected_ports.append(ports[port_idx])
        except ValueError:
            # Default to first port if invalid input
            print("Invalid input, using first port")
            if ports:
                selected_ports = [ports[0]]
    
    # Add each selected port
    for port in selected_ports:
        baud_rate = find_working_baud_rate(port)
        if baud_rate:
            monitor.add_device(port, baud_rate)
    
    if not monitor.devices:
        print("No GPS devices connected!")
        return
    
    # Main display loop
    try:
        last_screen_clear = 0
        screen_clear_interval = 1.0  # Clear screen once per second
        
        print("\nGPS Monitor running. Press Ctrl+C to exit.")
        
        while True:
            # Clear screen periodically
            current_time = time.time()
            if current_time - last_screen_clear >= screen_clear_interval:
                os.system('cls' if os.name == 'nt' else 'clear')
                last_screen_clear = current_time
                
                # Print header
                print("=" * 60)
                print(f"DIRECT GPS STATUS MONITOR - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                print("=" * 60)
                
                # Get a snapshot of devices with lock to avoid dict changes during iteration
                with monitor.lock:
                    devices_snapshot = {k: v.copy() for k, v in monitor.devices.items()}
                
                # Show device info
                device_list = list(devices_snapshot.items())
                
                if device_list:
                    print(f"Monitoring {len(device_list)} GPS devices:")
                    
                    for idx, (port, data) in enumerate(device_list):
                        # Check connection
                        if not data.get('connected', False):
                            print(f"\nDEVICE {idx+1}: {port} - DISCONNECTED")
                            continue
                        
                        # Format position
                        lat = data.get('lat')
                        lon = data.get('lon')
                        pos_str = f"{lat:.7f}, {lon:.7f}" if lat is not None and lon is not None else "No position"
                        
                        # Format satellites
                        sats_used = data.get('satellites_used', 0)
                        sats_visible = data.get('satellites_visible', 0)
                        sats_str = f"{sats_used}/{sats_visible} satellites"
                        
                        # Format HDOP
                        hdop = data.get('hdop', 0.0)
                        hdop_str = f"HDOP: {hdop:.2f}"
                        
                        # Format speed
                        speed = data.get('speed', None)
                        speed_str = f"{speed:.2f} m/s" if speed is not None else "Unknown"
                        
                        # Format heading
                        heading = data.get('track')
                        heading_str = f"{heading:.1f}°" if heading is not None else "Unknown"
                        
                        # Count NMEA sentences
                        sentence_counts = data.get('sentences', {})
                        
                        # Device status
                        print(f"\nDEVICE {idx+1}: {port} @ {data.get('baud_rate')} baud")
                        fix_mode_display = monitor.get_fix_mode_display(data)
                        print(f"  Status: {fix_mode_display}")
                        print(f"  Position: {pos_str}")
                        print(f"  Satellites: {sats_str} | {hdop_str}")
                        print(f"  Speed: {speed_str} | Heading: {heading_str}")
                        
                        # Show sentence counts if available
                        if sentence_counts:
                            print("  NMEA sentences:")
                            for stype, count in sorted(sentence_counts.items()):
                                print(f"    {stype}: {count}")
                    
                    # Calculate heading between devices if we have two
                    if len(device_list) >= 2:
                        p1, data1 = device_list[0]
                        p2, data2 = device_list[1]
                        
                        lat1, lon1 = data1.get('lat'), data1.get('lon')
                        lat2, lon2 = data2.get('lat'), data2.get('lon')
                        
                        if None not in (lat1, lon1, lat2, lon2):
                            # Calculate bearing from device 2 to device 1 (rear to front)
                            bearing = monitor.calculate_bearing(lat2, lon2, lat1, lon1)
                            dist = haversine_distance(lat1, lon1, lat2, lon2)
                            print(f"\nDual-GPS heading: {bearing:.1f}° | Distance: {dist:.2f}m")
                else:
                    print("\nNo GPS devices found or all disconnected.")
            
            # Small sleep to avoid CPU overuse
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        monitor.close()

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two points using haversine formula"""
    R = 6371000  # Earth radius in meters
    
    # Convert decimal degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Haversine formula
    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c
    
    return distance

if __name__ == "__main__":
    main()