#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/nmea_parser.py

import subprocess
import signal
import sys
import time
import re
from datetime import datetime

# Control flag
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\nStopping NMEA parser...")
    running = False

# NMEA sentence parsers
def parse_gga(sentence):
    """Parse GGA sentence (position and fix data)"""
    parts = sentence.split(',')
    if len(parts) < 15:
        return None
    
    # Fix quality from field 6:
    # 0=no fix, 1=GPS, 2=DGPS, 4=RTK Fix, 5=RTK Float
    fix_types = {
        '0': 'NO FIX',
        '1': 'GPS FIX',
        '2': 'DGPS',
        '3': 'PPS',
        '4': 'RTK FIX',
        '5': 'RTK FLOAT',
        '6': 'ESTIMATED'
    }
    
    # Extract data
    time_str = parts[1]
    lat = float(parts[2]) / 100 if parts[2] else None
    lat_dir = parts[3]
    lon = float(parts[4]) / 100 if parts[4] else None
    lon_dir = parts[5]
    fix_quality = fix_types.get(parts[6], f"UNKNOWN({parts[6]})")
    satellites = int(parts[7]) if parts[7] else 0
    hdop = float(parts[8]) if parts[8] else 0.0
    altitude = float(parts[9]) if parts[9] else 0.0
    
    # Convert DDMM.MMMM to decimal degrees
    if lat is not None:
        deg = int(lat)
        min_part = lat - deg
        lat = deg + (min_part * 100) / 60
        if lat_dir == 'S':
            lat = -lat
            
    if lon is not None:
        deg = int(lon)
        min_part = lon - deg
        lon = deg + (min_part * 100) / 60
        if lon_dir == 'W':
            lon = -lon
    
    return {
        'time': time_str,
        'lat': lat,
        'lon': lon,
        'fix_quality': fix_quality,
        'satellites': satellites,
        'hdop': hdop,
        'altitude': altitude
    }

def parse_rmc(sentence):
    """Parse RMC sentence (position, velocity, time)"""
    parts = sentence.split(',')
    if len(parts) < 12:
        return None
    
    # Extract heading and speed
    speed_knots = float(parts[7]) if parts[7] else 0.0
    heading = float(parts[8]) if parts[8] else 0.0
    date_str = parts[9] if parts[9] else ""
    
    return {
        'speed': speed_knots * 0.514444,  # Convert knots to m/s
        'heading': heading,
        'date': date_str
    }

def main():
    global running
    
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting NMEA Parser for GPS Status...")
    print("Press Ctrl+C to exit")
    print("-" * 60)
    
    # Track latest data
    latest_data = {
        'position': None,
        'speed': None,
        'heading': None,
        'fix_quality': 'UNKNOWN',
        'satellites': 0,
        'last_update': None
    }
    
    # Start gpsmon
    try:
        process = subprocess.Popen(
            ["gpsmon", "-a", "-n"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        
        last_status_print = time.time()
        
        while running:
            line = process.stdout.readline().strip()
            if not line:
                if process.poll() is not None:
                    print("gpsmon process has ended")
                    break
                continue
                
            # Extract the actual NMEA sentence (ignoring prefixed numbers)
            match = re.search(r'\(\d+\)\s+(\$[A-Z]+,.+)$', line)
            if match:
                nmea = match.group(1)
                
                # Process specific sentence types
                if nmea.startswith('$GPGGA'):
                    gga_data = parse_gga(nmea)
                    if gga_data:
                        latest_data['position'] = (gga_data['lat'], gga_data['lon'])
                        latest_data['fix_quality'] = gga_data['fix_quality']
                        latest_data['satellites'] = gga_data['satellites']
                        latest_data['hdop'] = gga_data['hdop']
                        latest_data['altitude'] = gga_data['altitude']
                        latest_data['last_update'] = datetime.now()
                        
                elif nmea.startswith('$GPRMC'):
                    rmc_data = parse_rmc(nmea)
                    if rmc_data:
                        latest_data['speed'] = rmc_data['speed']
                        latest_data['heading'] = rmc_data['heading']
                        latest_data['last_update'] = datetime.now()
            
            # Print status every second
            now = time.time()
            if now - last_status_print >= 1:
                # Clear screen
                print("\033[H\033[J", end="")
                
                # Print current status
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"=== GPS STATUS @ {timestamp} ===")
                
                # Fix quality with color
                fix_status = latest_data['fix_quality']
                if fix_status == 'RTK FIX':
                    fix_indicator = "🟢 RTK FIX"
                elif fix_status == 'RTK FLOAT':
                    fix_indicator = "🟡 RTK FLOAT"
                elif fix_status in ['DGPS', '3D FIX']:
                    fix_indicator = "🔵 " + fix_status
                else:
                    fix_indicator = "🔴 " + fix_status
                
                print(f"Status: {fix_indicator}")
                
                # Position
                if latest_data['position']:
                    lat, lon = latest_data['position']
                    print(f"Position: {lat:.7f}, {lon:.7f}")
                else:
                    print("Position: Unknown")
                
                # Other data
                print(f"Heading: {latest_data['heading']:.1f}°")
                print(f"Speed: {latest_data['speed']:.2f} m/s")
                print(f"Satellites: {latest_data['satellites']}")
                print(f"HDOP: {latest_data.get('hdop', 0.0):.2f}")
                print(f"Altitude: {latest_data.get('altitude', 0.0):.2f}m")
                
                # Data freshness
                if latest_data['last_update']:
                    age = (datetime.now() - latest_data['last_update']).total_seconds()
                    print(f"Data age: {age:.1f} seconds")
                
                last_status_print = now
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'process' in locals():
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()

if __name__ == "__main__":
    main()