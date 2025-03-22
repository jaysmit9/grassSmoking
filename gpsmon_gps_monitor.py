#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/gpsmon_gps_monitor.py

import subprocess
import time
import threading
import re
import json
import os
import signal
import curses
import sys
import argparse
from datetime import datetime

# Configuration
CONFIG = {
    "update_interval": 0.2,
    "clear_screen": True,
    "gps_tool": "gpsmon",  # Options: "gpspipe", "gpsmon"
    "gps_timeout": 5.0,
    "debug": False
}

# Global state
running = True
gps_data = {
    "rover": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_mode": None, "last_update": 0,
        "satellites_used": 0, "satellites_visible": 0,
        "precision": {"h": 0.0, "v": 0.0}, "pdop": 0.0,
        "device_path": None
    },
    "base": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_mode": None, "last_update": 0,
        "satellites_used": 0, "satellites_visible": 0, 
        "precision": {"h": 0.0, "v": 0.0}, "pdop": 0.0,
        "device_path": None
    }
}

# Fix quality descriptions
FIX_MODES = {
    'NO DATA': {'value': 0, 'desc': 'No data', 'color': 'red'},
    'NO FIX': {'value': 1, 'desc': 'No fix', 'color': 'red'},
    '2D FIX': {'value': 2, 'desc': '2D fix', 'color': 'yellow'},
    '3D FIX': {'value': 3, 'desc': '3D fix', 'color': 'green'},
    'GPS': {'value': 1, 'desc': 'GPS SPS mode', 'color': 'yellow'},
    'DGPS': {'value': 2, 'desc': 'Differential GPS', 'color': 'green'},
    'PPS': {'value': 3, 'desc': 'PPS fix', 'color': 'green'},
    'RTK_FIX': {'value': 4, 'desc': 'RTK Fixed', 'color': 'green'},
    'RTK_FLOAT': {'value': 5, 'desc': 'RTK Float', 'color': 'yellow'},
    'DR': {'value': 6, 'desc': 'Dead Reckoning', 'color': 'red'},
    'GNSS+DR': {'value': 7, 'desc': 'GNSS+DR', 'color': 'yellow'},
    'TIME_ONLY': {'value': 8, 'desc': 'Time only', 'color': 'red'},
    'SIM': {'value': 9, 'desc': 'Simulated', 'color': 'red'},
    'WAAS': {'value': 10, 'desc': 'WAAS/SBAS', 'color': 'green'},
    'ESTIMATED': {'value': 11, 'desc': 'Estimated', 'color': 'red'},
    'MANUAL': {'value': 12, 'desc': 'Manual', 'color': 'red'}
}

# Device assignments
device_roles = {
    "rover": None,
    "base": None
}

class GPSToolParser:
    """Parser for GPS tool output"""
    
    def __init__(self, tool="gpspipe", debug=False):
        self.tool = tool
        self.debug = debug
        self.process = None
        self.devices = {}
        
    def log(self, message):
        """Log debug messages"""
        if self.debug:
            print(f"[DEBUG] {message}")
            
    def start(self):
        """Start the GPS monitoring tool subprocess"""
        if self.tool == "gpspipe":
            # gpspipe with JSON output is easiest to parse
            cmd = ["gpspipe", "-w", "-n", "-p"]
            self.log(f"Starting: {' '.join(cmd)}")
            self.process = subprocess.Popen(
                cmd, 
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1  # Line buffered
            )
            return True
        elif self.tool == "gpsmon":
            # Use gpsmon -n for NMEA output which is much easier to parse
            cmd = ["gpsmon", "-n"]  # Output raw NMEA sentences
            self.log(f"Starting: {' '.join(cmd)}")
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE,
                universal_newlines=True,
                bufsize=1
            )
            return True
        else:
            print(f"Unknown GPS tool: {self.tool}")
            return False
            
    def stop(self):
        """Stop the subprocess"""
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
            self.process = None
            
    def process_line(self, line):
        """Process a line of output from the GPS tool"""
        if self.tool == "gpspipe":
            return self._process_gpspipe_line(line)
        elif self.tool == "gpsmon":
            return self._process_gpsmon_line(line)
        return None
            
    def _process_gpspipe_line(self, line):
        """Process a line of JSON output from gpspipe"""
        try:
            if not line.strip():
                return None
                
            # Clean the line - sometimes gpspipe adds non-JSON text
            json_start = line.find("{")
            if json_start == -1:
                return None
                
            clean_line = line[json_start:]
            # Remove any trailing non-JSON characters
            while clean_line and clean_line[-1] not in "]}":
                clean_line = clean_line[:-1]
                
            if not clean_line:
                return None
                
            # Parse the JSON
            data = json.loads(clean_line)
            
            # Process based on message class
            if data.get("class") == "TPV":
                # Position, time, velocity report
                device_path = data.get("device", "unknown")
                
                # Create device entry if it doesn't exist
                if device_path not in self.devices:
                    self.devices[device_path] = {}
                    
                # Define fix mode based on mode and status
                mode = data.get("mode", 0)
                status = data.get("status", 0)
                fix_mode = self._determine_fix_mode(data)
                
                # Record position data
                self.devices[device_path].update({
                    "last_update": time.time(),
                    "lat": data.get("lat"),
                    "lon": data.get("lon"),
                    "alt": data.get("alt"),
                    "speed": data.get("speed"),
                    "track": data.get("track"),  # Heading/course
                    "fix_mode": fix_mode,
                    "eph": data.get("eph", 0.0),  # Horizontal precision
                    "epv": data.get("epv", 0.0),  # Vertical precision
                    "has_fix": mode >= 2
                })
                
                return {"type": "position", "device": device_path}
                
            elif data.get("class") == "SKY":
                # Satellite data
                device_path = data.get("device", "unknown")
                
                # Create device entry if it doesn't exist
                if device_path not in self.devices:
                    self.devices[device_path] = {}
                    
                # Process satellites
                satellites = data.get("satellites", [])
                satellites_used = sum(1 for s in satellites if s.get("used"))
                
                # Update satellite data
                self.devices[device_path].update({
                    "last_update": time.time(),
                    "satellites_used": satellites_used,
                    "satellites_visible": len(satellites),
                    "hdop": data.get("hdop", 0.0),
                    "vdop": data.get("vdop", 0.0),
                    "pdop": data.get("pdop", 0.0)
                })
                
                return {"type": "sky", "device": device_path}
                
            elif data.get("class") == "DEVICES":
                # Available devices
                for device in data.get("devices", []):
                    path = device.get("path")
                    if path:
                        if path not in self.devices:
                            self.devices[path] = {}
                        self.devices[path].update({
                            "driver": device.get("driver"),
                            "activated": device.get("activated", ""),
                            "last_update": time.time()
                        })
                
                return {"type": "devices", "count": len(data.get("devices", []))}
                
            return None
        except json.JSONDecodeError:
            self.log(f"JSON parse error: {line}")
            return None
        except Exception as e:
            self.log(f"Error processing gpspipe line: {e}")
            return None
            
    def _process_gpsmon_line(self, line):
        """Process a line of output from gpsmon"""
        try:
            # This is more complex as gpsmon output is meant for display, not parsing
            # We'll look for specific patterns
            
            # Check for NMEA GGA sentence (has fix quality)
            if "$GPGGA" in line or "$GNGGA" in line:
                fields = line.strip().split(',')
                if len(fields) >= 7:
                    # Extract fix quality
                    quality = fields[6]
                    device_path = "gpsmon_device"  # gpsmon doesn't specify the device path in output
                    
                    if device_path not in self.devices:
                        self.devices[device_path] = {}
                    
                    # Map quality to fix mode
                    quality_map = {
                        '0': 'NO FIX',
                        '1': 'GPS',
                        '2': 'DGPS',
                        '4': 'RTK_FIX',
                        '5': 'RTK_FLOAT'
                    }
                    fix_mode = quality_map.get(quality, "UNKNOWN")
                    
                    # Extract lat/lon if available
                    try:
                        if len(fields) >= 4 and fields[2] and fields[4]:
                            lat_raw = fields[2]
                            lat_dir = fields[3]
                            lat_deg = float(lat_raw[0:2])
                            lat_min = float(lat_raw[2:])
                            lat = lat_deg + lat_min/60
                            if lat_dir == 'S':
                                lat = -lat
                                
                            lon_raw = fields[4]
                            lon_dir = fields[5]
                            lon_deg = float(lon_raw[0:3])
                            lon_min = float(lon_raw[3:])
                            lon = lon_deg + lon_min/60
                            if lon_dir == 'W':
                                lon = -lon
                                
                            self.devices[device_path]["lat"] = lat
                            self.devices[device_path]["lon"] = lon
                            self.devices[device_path]["fix_mode"] = fix_mode
                            self.devices[device_path]["last_update"] = time.time()
                    except:
                        pass
                        
                    return {"type": "position", "device": device_path}
                    
            # Look for satellite count
            sat_match = re.search(r'Sats used:\s*(\d+)', line)
            if sat_match:
                device_path = "gpsmon_device"
                if device_path not in self.devices:
                    self.devices[device_path] = {}
                    
                self.devices[device_path]["satellites_used"] = int(sat_match.group(1))
                self.devices[device_path]["last_update"] = time.time()
                return {"type": "satellites", "device": device_path}
                
            return None
        except Exception as e:
            self.log(f"Error processing gpsmon line: {e}")
            return None
            
    def _determine_fix_mode(self, data):
        """Determine detailed fix mode from gpspipe data"""
        # Start with basic mode
        mode = data.get("mode", 0)
        mode_map = {0: 'NO DATA', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX'}
        fix_mode = mode_map.get(mode, 'UNKNOWN')
        
        # Check status for RTK/DGPS info
        status = data.get("status", 0)
        if status == 2:
            fix_mode = "DGPS"
        elif status == 3:
            fix_mode = "RTK_FIX"
        elif status == 4:
            fix_mode = "RTK_FLOAT"
            
        # Look for RTK indicators in raw NMEA data
        if "nmea" in data:
            nmea = data.get("nmea", "")
            if "$GNGGA" in nmea or "$GPGGA" in nmea:
                gga_parts = nmea.split(',')
                if len(gga_parts) >= 7:
                    quality = gga_parts[6]
                    quality_map = {
                        '0': 'NO FIX',
                        '1': 'GPS',
                        '2': 'DGPS',
                        '4': 'RTK_FIX',
                        '5': 'RTK_FLOAT'
                    }
                    if quality in quality_map:
                        fix_mode = quality_map[quality]
        
        return fix_mode

    def assign_devices(self):
        """Determine which device is rover and which is base"""
        global device_roles
        
        if not self.devices:
            return
            
        # If roles are already assigned and devices still exist, keep them
        if (device_roles["rover"] in self.devices and
            device_roles["base"] in self.devices):
            return
            
        # Find devices with RTK capability - prefer those for rover
        rtk_devices = []
        other_devices = []
        
        for path, info in self.devices.items():
            fix_mode = info.get("fix_mode", "UNKNOWN")
            if fix_mode in ["RTK_FIX", "RTK_FLOAT"]:
                rtk_devices.append(path)
            else:
                other_devices.append(path)
        
        # If we have RTK devices, use the first one as rover
        if rtk_devices:
            device_roles["rover"] = rtk_devices[0]
            # Use another RTK device as base if available, otherwise use first other device
            if len(rtk_devices) > 1:
                device_roles["base"] = rtk_devices[1]
            elif other_devices:
                device_roles["base"] = other_devices[0]
            else:
                # If only one device, use it for both roles
                device_roles["base"] = rtk_devices[0]
        elif other_devices:
            # No RTK devices, use whatever we have
            device_roles["rover"] = other_devices[0]
            if len(other_devices) > 1:
                device_roles["base"] = other_devices[1]
            else:
                device_roles["base"] = other_devices[0]
        
        # Log the assignments
        if device_roles["rover"]:
            print(f"Assigned rover role to: {device_roles['rover']}")
        if device_roles["base"]:
            print(f"Assigned base role to: {device_roles['base']}")

def gps_reader_thread(parser):
    """Thread that reads from GPS tool and updates data"""
    global running, gps_data
    
    while running:
        try:
            if not parser.process:
                if not parser.start():
                    print("Failed to start GPS tool. Retrying in 5 seconds...")
                    time.sleep(5)
                    continue
            
            # Read line from the process
            line = parser.process.stdout.readline()
            if not line:
                # Check if process has ended
                if parser.process.poll() is not None:
                    print(f"GPS tool process exited with code {parser.process.returncode}")
                    parser.process = None
                    time.sleep(1)
                continue
                
            # Process the line
            result = parser.process_line(line)
            if not result:
                continue
                
            # Assign devices to roles if needed
            parser.assign_devices()
            
            # Update the global GPS data structure
            for role in ["rover", "base"]:
                if device_roles[role] in parser.devices:
                    device_info = parser.devices[device_roles[role]]
                    
                    # Update position data if available
                    if "lat" in device_info and "lon" in device_info:
                        gps_data[role]["lat"] = device_info["lat"]
                        gps_data[role]["lon"] = device_info["lon"]
                        
                    # Update other fields if available
                    if "fix_mode" in device_info:
                        gps_data[role]["fix_mode"] = device_info["fix_mode"]
                    if "track" in device_info:
                        gps_data[role]["heading"] = device_info["track"]
                    if "speed" in device_info:
                        gps_data[role]["speed"] = device_info["speed"]
                        
                    # Update satellite data if available
                    if "satellites_used" in device_info:
                        gps_data[role]["satellites_used"] = device_info["satellites_used"]
                    if "satellites_visible" in device_info:
                        gps_data[role]["satellites_visible"] = device_info["satellites_visible"]
                        
                    # Update precision data if available
                    if "eph" in device_info:
                        gps_data[role]["precision"]["h"] = device_info["eph"]
                    if "epv" in device_info:
                        gps_data[role]["precision"]["v"] = device_info["epv"]
                    if "pdop" in device_info:
                        gps_data[role]["pdop"] = device_info["pdop"]
                        
                    # Update device path and timestamp
                    gps_data[role]["device_path"] = device_roles[role]
                    gps_data[role]["last_update"] = device_info.get("last_update", time.time())
            
        except Exception as e:
            print(f"Error in GPS reader thread: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)

def get_fix_status_indicator(fix_mode):
    """Return colored status indicator based on fix mode"""
    if fix_mode in ['RTK_FIX']:
        return "[RTK FIXED] 🟢"
    elif fix_mode in ['RTK_FLOAT']:
        return "[RTK FLOAT] 🟡"
    elif fix_mode in ['DGPS', 'WAAS']:
        return "[DGPS] 🔵"
    elif fix_mode == '3D FIX':
        return "[3D FIX] 🔵"
    elif fix_mode == '2D FIX':
        return "[2D FIX] 🟠"
    elif fix_mode in ['NO FIX', 'NO DATA']:
        return "[NO FIX] 🔴"
    else:
        return f"[{fix_mode}]"

def format_position(lat, lon):
    """Format a lat/lon position for display"""
    if lat is None or lon is None:
        return "No position"
    return f"{lat:.7f}, {lon:.7f}"

def print_gps_status(stdscr=None):
    """Print current GPS status to terminal or curses window"""
    if stdscr:
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        row = 0
    else:
        if CONFIG["clear_screen"]:
            os.system('cls' if os.name == 'nt' else 'clear')
    
    # Prepare lines to print
    lines = []
    lines.append(f"============================================================")
    lines.append(f"GPS MONITOR using {CONFIG['gps_tool']} - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"============================================================")
    lines.append(f"Monitoring devices:")
    
    # Print rover info
    rover_status = get_fix_status_indicator(gps_data['rover']['fix_mode'] or "NO DATA")
    lines.append(f"\nROVER GPS: {gps_data['rover']['device_path'] or 'Not detected'}")
    lines.append(f"  Status: {rover_status}")
    lines.append(f"  Position: {format_position(gps_data['rover']['lat'], gps_data['rover']['lon'])}")
    lines.append(f"  Satellites: {gps_data['rover']['satellites_used']}/{gps_data['rover']['satellites_visible']} satellites | HDOP: {gps_data['rover'].get('hdop', 0.0):.2f}")
    lines.append(f"  Speed: {gps_data['rover']['speed'] or 0.0:.2f} m/s | Heading: {gps_data['rover']['heading'] or 'Unknown'}")
    
    # Print base info
    base_status = get_fix_status_indicator(gps_data['base']['fix_mode'] or "NO DATA")
    lines.append(f"\nBASE GPS: {gps_data['base']['device_path'] or 'Not detected'}")
    lines.append(f"  Status: {base_status}")
    lines.append(f"  Position: {format_position(gps_data['base']['lat'], gps_data['base']['lon'])}")
    lines.append(f"  Satellites: {gps_data['base']['satellites_used']}/{gps_data['base']['satellites_visible']} satellites | HDOP: {gps_data['base'].get('hdop', 0.0):.2f}")
    
    # Calculate distance between GPSs if both positions are available
    if (gps_data["rover"]["lat"] is not None and 
        gps_data["base"]["lat"] is not None):
        import math
        
        # Convert to radians
        lat1, lon1 = map(math.radians, [gps_data["rover"]["lat"], gps_data["rover"]["lon"]])
        lat2, lon2 = map(math.radians, [gps_data["base"]["lat"], gps_data["base"]["lon"]])
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        distance = c * 6371000  # Radius of earth in meters
        
        lines.append(f"\nGPS separation: {distance:.2f}m")
    
    # Print each line
    if stdscr:
        for i, line in enumerate(lines):
            if i < height-1:  # Avoid writing to the bottom line
                stdscr.addstr(i, 0, line[:width-1])  # Avoid writing to the last column
        stdscr.refresh()
    else:
        for line in lines:
            print(line)

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    global running
    print("\nStopping GPS monitor...")
    running = False

def console_ui_thread():
    """Thread that handles console UI"""
    global running
    
    try:
        # Initialize curses
        stdscr = curses.initscr()
        curses.start_color()
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)  # Non-blocking input
        
        # Main UI loop
        while running:
            # Handle keyboard input
            try:
                key = stdscr.getch()
                if key != -1:  # -1 means no key pressed
                    if key == ord('q'):  # Quit
                        running = False
            except Exception as e:
                print(f"Error handling keyboard input: {e}")
            
            # Update display
            print_gps_status(stdscr)
            
            # Sleep
            time.sleep(CONFIG["update_interval"])
            
    except Exception as e:
        print(f"Error in UI thread: {e}")
    finally:
        # Clean up curses
        try:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()
        except:
            pass

def console_mode():
    """Run GPS monitor in simple console mode without curses"""
    global running
    
    try:
        while running:
            print_gps_status()
            time.sleep(CONFIG["update_interval"])
            
    except KeyboardInterrupt:
        running = False
    except Exception as e:
        print(f"Error in console mode: {e}")

def main():
    global running, CONFIG
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="GPS Monitor using external tools")
    parser.add_argument("-t", "--tool", choices=["gpspipe", "gpsmon"], default=CONFIG["gps_tool"], 
                        help="GPS tool to use")
    parser.add_argument("-i", "--interval", type=float, default=CONFIG["update_interval"], 
                        help="Update interval")
    parser.add_argument("-n", "--no-clear", action="store_true", 
                        help="Don't clear screen between updates")
    parser.add_argument("-c", "--console", action="store_true", 
                        help="Use simple console mode (no curses)")
    parser.add_argument("-d", "--debug", action="store_true", 
                        help="Enable debug output")
    args = parser.parse_args()
    
    # Update config with command line args
    CONFIG["gps_tool"] = args.tool
    CONFIG["update_interval"] = args.interval
    CONFIG["clear_screen"] = not args.no_clear
    CONFIG["debug"] = args.debug
    
    # Set up signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    print(f"Starting GPS monitor using {CONFIG['gps_tool']}...")
    
    # Initialize the GPS tool parser
    parser = GPSToolParser(tool=CONFIG["gps_tool"], debug=CONFIG["debug"])
    
    # Start GPS reader thread
    gps_thread = threading.Thread(target=gps_reader_thread, args=(parser,))
    gps_thread.daemon = True
    gps_thread.start()
    
    try:
        # Run UI thread
        if args.console:
            console_mode()
        else:
            console_ui_thread()
    except Exception as e:
        print(f"Error in main thread: {e}")
    finally:
        # Clean up
        running = False
        parser.stop()
        print("GPS Monitor stopped.")

if __name__ == "__main__":
    main()