#!/usr/bin/env python3

import time
import argparse
import sys
import json
from datetime import datetime
import socket

class GpsdClient:
    """A simple GPSD client that doesn't rely on the gps package"""
    
    def __init__(self, host='localhost', port=2947, debug=False):
        """Initialize connection to GPSD"""
        self.host = host
        self.port = port
        self.socket = None
        self.debug = debug
        self.connect()
        
    def log(self, message):
        """Print debug messages if debug is enabled"""
        if self.debug:
            print(f"[DEBUG] {message}")
        
    def connect(self):
        """Connect to the GPSD daemon"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            
            # Switch GPSD to JSON mode
            watch_command = b'?WATCH={"enable":true,"json":true}\n'
            self.socket.send(watch_command)
            self.log(f"Sent WATCH command: {watch_command}")
            
            # Read initial response
            initial_response = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"Initial response: {initial_response}")
            
            return True
        except Exception as e:
            print(f"Error connecting to GPSD: {e}")
            return False
            
    def get_report(self):
        """Get a GPSD report"""
        if not self.socket:
            return None
            
        try:
            self.log("Waiting for data...")
            # Set timeout to prevent infinite blocking
            self.socket.settimeout(5.0)
            data = self.socket.recv(4096).decode('utf-8').strip()
            
            if not data:
                self.log("No data received")
                return None
                
            self.log(f"Received data: {data}")
                
            # Handle multiple JSON objects
            for line in data.split('\n'):
                if not line.strip():
                    continue
                try:
                    return json.loads(line)
                except json.JSONDecodeError as e:
                    self.log(f"JSON decode error: {e} for line: {line}")
                    continue
                    
            return None
        except socket.timeout:
            self.log("Socket timeout occurred")
            return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    def send_poll(self):
        """Send a POLL command to request data"""
        if self.socket:
            try:
                self.socket.send(b'?POLL;\n')
                self.log("Sent POLL command")
                return True
            except Exception as e:
                self.log(f"Error sending POLL: {e}")
                return False
        return False

def format_lat_lon(value, pos, neg):
    """Format latitude/longitude as text string"""
    if value > 0:
        return f"{abs(value):.7f}° {pos}"
    else:
        return f"{abs(value):.7f}° {neg}"  # Fixed double colon

def main():
    parser = argparse.ArgumentParser(description="Read GPS data from GPSD daemon")
    parser.add_argument("-H", "--host", type=str, default="localhost", help="GPSD host (default: localhost)")
    parser.add_argument("-p", "--port", type=int, default=2947, help="GPSD port (default: 2947)")
    parser.add_argument("-j", "--json", action="store_true", help="Output in JSON format")
    parser.add_argument("-i", "--interval", type=float, default=1.0, help="Update interval in seconds")
    parser.add_argument("-o", "--output", type=str, help="Output file (optional)")
    parser.add_argument("-d", "--debug", action="store_true", help="Enable debug output")
    parser.add_argument("-c", "--check", action="store_true", help="Check GPS device status and exit")
    args = parser.parse_args()

    gpsd = GpsdClient(host=args.host, port=args.port, debug=args.debug)
    
    # Check mode - just print device status and exit
    if args.check:
        print(f"Connecting to GPSD at {args.host}:{args.port}...")
        gpsd.socket.send(b'?DEVICES;\n')
        time.sleep(0.5)
        
        data = gpsd.socket.recv(4096).decode('utf-8').strip()
        print(f"GPSD Response: {data}")
        
        gpsd.socket.send(b'?WATCH={"enable":true,"json":true};\n')
        time.sleep(0.5)
        
        data = gpsd.socket.recv(4096).decode('utf-8').strip()
        print(f"WATCH Response: {data}")
        
        print("Check complete. Run without --check to start continuous monitoring.")
        return
    
    outfile = None
    if args.output:
        try:
            outfile = open(args.output, 'w')
        except Exception as e:
            print(f"Error opening output file: {e}")
            sys.exit(1)

    try:
        print(f"Reading GPS data from {args.host}:{args.port}... Press Ctrl+C to exit.")
        poll_count = 0
        report_count = 0
        
        while True:
            # Send a poll every 5 iterations if we're not getting data
            if report_count == 0 and poll_count % 5 == 0:
                gpsd.send_poll()
                
            report = gpsd.get_report()
            poll_count += 1
            
            if report is None:
                if poll_count % 10 == 0:
                    print("No data received, reconnecting...")
                    gpsd.connect()
                time.sleep(args.interval)
                continue
            
            # Only process TPV (Time-Position-Velocity) reports
            if report.get('class') != 'TPV':
                if args.debug:
                    print(f"Skipping non-TPV report: {report.get('class')}")
                continue

            report_count += 1
            current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            # Extract relevant GPS data
            data = {
                "time": current_time,
                "gps_time": report.get('time', 'n/a'),
                "lat": report.get('lat', 0.0),
                "lon": report.get('lon', 0.0),
                "alt": report.get('alt', 0.0),
                "track": report.get('track', 0.0),  # heading in degrees
                "speed": report.get('speed', 0.0),  # speed in m/s
                "climb": report.get('climb', 0.0),  # rate of climb in m/s
                "mode": report.get('mode', 0)  # 0=no data, 1=no fix, 2=2D fix, 3=3D fix
            }

            # Output in JSON or human-readable format
            if args.json:
                output = json.dumps(data)
                print(output)
                if outfile:
                    outfile.write(output + "\n")
                    outfile.flush()
            else:
                mode_map = {0: 'NO DATA', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX'}
                mode_text = mode_map.get(data['mode'], f"MODE {data['mode']}")
                lat_str = format_lat_lon(data['lat'], 'N', 'S')
                lon_str = format_lat_lon(data['lon'], 'E', 'W')
                
                output = (
                    f"TIME: {current_time} | GPS TIME: {data['gps_time']}\n"
                    f"POS: {lat_str}, {lon_str} | ALT: {data['alt']:.1f}m\n"
                    f"HDG: {data['track']:.1f}° | SPD: {data['speed']*3.6:.1f}km/h | MODE: {mode_text}\n"
                    f"{'-'*50}"
                )
                print(output)
                if outfile:
                    outfile.write(output + "\n")
                    outfile.flush()
                    
            time.sleep(args.interval)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if outfile:
            outfile.close()
        if gpsd.socket:
            gpsd.socket.close()

if __name__ == "__main__":
    main()