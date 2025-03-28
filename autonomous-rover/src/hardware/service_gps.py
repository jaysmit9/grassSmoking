#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/hardware/service_gps.py

import serial
import os
import time
import math
import sys
import glob
import json
from datetime import datetime
import threading
import argparse
from collections import deque
from flask import Flask, jsonify, request
from geopy.distance import geodesic  # Add this import

# ANSI color codes for prettier output
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_MAGENTA = "\033[35m"
COLOR_CYAN = "\033[36m"
COLOR_RED = "\033[31m"
COLOR_BOLD = "\033[1m"

# Status icons
ICON_RTK_FIXED = "🟢"
ICON_RTK_FLOAT = "🟡"
ICON_3D_FIX = "🔵"
ICON_NO_FIX = "🔴"
ICON_NO_DATA = "⚪"

class DirectGpsMonitor:
    """GPS Monitor that directly reads from serial ports"""
    
    def __init__(self):
        self.devices = {}  # Store device info by port name
        self.threads = {}  # Reader threads
        self.running = True
        self.lock = threading.Lock()  # For thread-safe device dictionary updates
        
        # For tracking position history (useful for heading calculation)
        self.position_history = {}
        self.history_length = 10
        
        # Track which devices receive RTCM corrections
        self.rtcm_detected = set()
    
    def find_serial_ports(self):
        """Find available serial ports"""
        if sys.platform.startswith('linux'):
            return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        elif sys.platform.startswith('win'):
            # Windows ports
            return ['COM%s' % (i + 1) for i in range(256)]
        else:
            return glob.glob('/dev/tty.*') + glob.glob('/dev/cu.*')
    
    def add_device(self, port, baud_rate=115200, device_name=None):
        """Add a GPS device to monitor"""
        # Create thread-safe device entry
        with self.lock:
            self.devices[port] = {
                'port': port,
                'name': device_name or f"GPS-{port.split('/')[-1]}",
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
                'hdop': 0.0,
                'update_rate': 0,
                'last_rates_time': time.time(),
                'sentence_counts_snapshot': {},
                'sentences': {},
                'rtcm_count': 0,  # Track RTCM message count
                'last_rtcm_time': 0
            }
            
            # Initialize position history for this device
            self.position_history[port] = deque(maxlen=self.history_length)
        
        # Start monitoring thread
        thread = threading.Thread(
            target=self._monitor_thread,
            args=(port, baud_rate),
            daemon=True
        )
        self.threads[port] = thread
        thread.start()
        
        print(f"Added GPS device: {port} ({device_name or 'unnamed'}) @ {baud_rate} baud")
        return True
    
    def _monitor_thread(self, port, baud_rate):
        """Thread to monitor a single GPS device"""
        ser = None
        buffer = ""
        binary_buffer = bytearray()
        
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
                        # Keep raw data for RTCM detection
                        binary_buffer.extend(data)
                        
                        # Check for RTCM messages (typically starts with 0xD3)
                        self._check_for_rtcm(port, binary_buffer)
                        
                        # Limit buffer size
                        if len(binary_buffer) > 4096:
                            binary_buffer = binary_buffer[-2048:]
                        
                        # Handle ASCII NMEA sentences
                        try:
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
                        except Exception as e:
                            # Not valid ASCII - likely binary data (RTCM)
                            pass
                    
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
    
    def _check_for_rtcm(self, port, buffer):
        """Check for RTCM message signatures in binary data"""
        # RTCM3 messages start with 0xD3 followed by reserved bits and length
        # We'll just look for the signature byte for simplicity
        rtcm_sig = b'\xD3'
        
        # Count occurrences
        count = buffer.count(rtcm_sig)
        if count > 0:
            with self.lock:
                if port in self.devices:
                    # Update RTCM count
                    self.devices[port]['rtcm_count'] = self.devices[port].get('rtcm_count', 0) + count
                    self.devices[port]['last_rtcm_time'] = time.time()
                    
                    # Add to RTCM-enabled devices
                    self.rtcm_detected.add(port)
    
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
                                
                                # Add to position history
                                if lat is not None and lon is not None:
                                    self.position_history[port].append({
                                        'lat': lat,
                                        'lon': lon,
                                        'time': time.time()
                                    })
                                
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
    
    def assign_device_roles(self):
        """Assign Front/Rear roles based on RTCM detection and RTK status"""
        with self.lock:
            devices = list(self.devices.keys())
            
            if len(devices) < 2:
                # Only one device, it's the "main" GPS
                if devices:
                    self.devices[devices[0]]['name'] = "Main GPS"
                return
            
            # First try: Find RTK devices (FLOAT or FIXED)
            rtk_devices = []
            for port in devices:
                rtk_mode = self.devices[port].get('rtk_mode')
                if rtk_mode in ['RTK_FLOAT', 'RTK_FIX']:
                    rtk_devices.append(port)
            
            # Second try: Find RTCM-receiving devices
            rtcm_devices = [port for port in devices if port in self.rtcm_detected]
            
            # Third try: Find highest quality
            best_quality_device = max(devices, key=lambda p: self.devices[p].get('gga_quality', 0))
            
            # Decision logic: RTK mode takes precedence, then RTCM, then quality
            if rtk_devices:
                front_device = rtk_devices[0]  # Use first RTK device as front
                print(f"Setting front GPS based on RTK status: {front_device}")
            elif rtcm_devices:
                front_device = rtcm_devices[0]  # Use first RTCM device as front
                print(f"Setting front GPS based on RTCM reception: {front_device}")
            else:
                front_device = best_quality_device
                print(f"Setting front GPS based on fix quality: {front_device}")
            
            # Set all other devices as rear or additional
            other_devices = [p for p in devices if p != front_device]
            
            # Set device names
            self.devices[front_device]['name'] = "Front GPS"
            self.devices[front_device]['is_front'] = True
            
            for i, port in enumerate(other_devices):
                if i == 0:
                    self.devices[port]['name'] = "Rear GPS"
                    self.devices[port]['is_front'] = False
                else:
                    self.devices[port]['name'] = f"GPS {i+1}"
                    self.devices[port]['is_front'] = False
                    
            # For debugging
            for port in devices:
                data = self.devices[port]
                print(f"Role: {data['name']}, RTK: {data.get('rtk_mode')}, RTCM: {'Yes' if port in self.rtcm_detected else 'No'}")
    
    def calculate_update_rates(self):
        """Calculate update rate for each GPS device"""
        current_time = time.time()
        
        with self.lock:
            for port, data in self.devices.items():
                # Calculate time since last rate calculation
                time_diff = current_time - data.get('last_rates_time', current_time)
                if time_diff >= 1.0:  # At least 1 second passed
                    # Get current and previous sentence counts
                    current_counts = data.get('sentences', {})
                    prev_counts = data.get('sentence_counts_snapshot', {})
                    
                    # Calculate differences for each sentence type
                    total_diff = 0
                    for sentence_type, count in current_counts.items():
                        prev = prev_counts.get(sentence_type, 0)
                        diff = count - prev
                        total_diff += diff
                    
                    # Calculate update rate in messages per second
                    data['update_rate'] = total_diff / time_diff if time_diff > 0 else 0
                    
                    # Update snapshot and timestamp
                    data['sentence_counts_snapshot'] = current_counts.copy()
                    data['last_rates_time'] = current_time
    
    def get_fix_mode_display(self, data):
        """Return colored status indicator based on device data"""
        # Check for RTK status
        rtk_mode = data.get('rtk_mode')
        quality = data.get('gga_quality', 0)
        
        # Build status text with color
        if rtk_mode == 'RTK_FIX':
            return f"{COLOR_GREEN}[RTK FIXED]{COLOR_RESET} {ICON_RTK_FIXED}"
        elif rtk_mode == 'RTK_FLOAT':
            return f"{COLOR_YELLOW}[RTK FLOAT]{COLOR_RESET} {ICON_RTK_FLOAT}"
        elif rtk_mode == 'DGPS':
            return f"{COLOR_BLUE}[DGPS]{COLOR_RESET} {ICON_3D_FIX}"
        elif rtk_mode == '3D_FIX':
            return f"{COLOR_BLUE}[3D FIX]{COLOR_RESET} {ICON_3D_FIX}"
        elif rtk_mode == 'NO_FIX':
            return f"{COLOR_RED}[NO FIX]{COLOR_RESET} {ICON_NO_FIX}"
        elif quality > 0:
            return f"{COLOR_CYAN}[FIX QUALITY: {quality}]{COLOR_RESET} {ICON_NO_DATA}"
        else:
            return f"{COLOR_RED}[NO DATA]{COLOR_RESET} {ICON_NO_DATA}"
    
    def close(self):
        """Clean shutdown of monitor and threads"""
        self.running = False
        
        # Wait for threads to finish
        for thread in self.threads.values():
            thread.join(timeout=1.0)
        
        print("GPS Monitor closed")

# Rest of the code remains similar, but we'll modify main()

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
                    try:
                        text = data.decode('ascii', errors='replace')
                        if '$GP' in text or '$GN' in text:
                            return True
                    except:
                        # Could be binary data (RTCM)
                        # Check for RTCM signature 0xD3
                        if b'\xD3' in data:
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

def draw_side_by_side_display(devices_data):
    """Create a side-by-side display of two GPS devices"""
    if not devices_data or len(devices_data) < 1:
        return "No GPS devices available"
        
    # Format for each column
    col_width = 40
    
    # Header
    display = []
    header = []
    divider = []
    
    # Sort devices by name to ensure consistent ordering (Front GPS first)
    sorted_devices = sorted(devices_data.items(), 
                          key=lambda x: 0 if x[1].get('name', '').startswith('Front') else 
                                       (1 if x[1].get('name', '').startswith('Rear') else 2))
    
    for port, data in sorted_devices:
        name = data.get('name', port)
        header.append(f"{name:^{col_width}}")
        divider.append("-" * col_width)
    
    display.append(" | ".join(header))
    display.append(" | ".join(divider))
    
    # Position info
    positions = []
    for port, data in sorted_devices:
        lat = data.get('lat')
        lon = data.get('lon')
        if lat is not None and lon is not None:
            pos_text = f"Lat: {lat:.7f} | Lon: {lon:.7f}"
            positions.append(f"{pos_text:^{col_width}}")
        else:
            positions.append(f"{'No position':^{col_width}}")
    
    display.append(" | ".join(positions))
    display.append(" | ".join(divider))
    
    # Status info with RTCM indication - STACKED VERSION
    statuses = []  # <-- ADD THIS LINE to initialize the list
    for port, data in sorted_devices:
        fix_mode = data.get('rtk_mode', 'NO_DATA')
        quality = data.get('gga_quality', 0)
        sats_used = data.get('satellites_used', 0)
        sats_visible = data.get('satellites_visible', 0)
        hdop = data.get('hdop', 0.0) or 0.0  # Ensure not None
        rtcm = "RTCM: YES" if data.get('rtcm_count', 0) > 0 else "RTCM: NO"
        update_rate = data.get('update_rate', 0)
        
        # Handle None values safely
        fix_mode_str = str(fix_mode) if fix_mode is not None else "Unknown"
        
        # Create a multi-line formatted status block
        status = [
            f"Status: {fix_mode_str}",
            f"{rtcm}",
            f"Quality: {quality}",
            f"Sats: {sats_used}/{sats_visible}",
            f"HDOP: {hdop:.2f}",
            f"Rate: {update_rate:.1f} Hz"
        ]
        
        # Center each line in the column
        statuses.append([line.center(col_width) for line in status])

    # Join the lines of status together
    # We need to transpose the data first
    status_rows = []
    for i in range(len(statuses[0])):  # For each row in the status blocks
        row = []
        for dev_status in statuses:
            row.append(dev_status[i])
        status_rows.append(" | ".join(row))

    # Add each row to display
    for row in status_rows:
        display.append(row)
    
    # Speed and heading
    movement = []
    for port, data in sorted_devices:
        speed = data.get('speed')
        track = data.get('track')
        
        speed_str = f"{speed:.2f} m/s" if speed is not None else "Unknown"
        heading_str = f"{track:.1f}°" if track is not None else "Unknown"
        
        mov_text = f"Speed: {speed_str} | Heading: {heading_str}"
        movement.append(f"{mov_text:^{col_width}}")
    
    display.append(" | ".join(movement))
    
    # Add dual-GPS calculation if we have front and rear
    if len(devices_data) >= 2:
        display.append("=" * (col_width * len(devices_data) + 3 * (len(devices_data) - 1)))
        
        # Get Front and Rear GPS
        front_gps = None
        rear_gps = None
        
        for port, data in devices_data.items():
            name = data.get('name', '')
            if name.startswith('Front'):
                front_gps = data
            elif name.startswith('Rear'):
                rear_gps = data
        
        if front_gps and rear_gps:
            lat1, lon1 = front_gps.get('lat'), front_gps.get('lon')
            lat2, lon2 = rear_gps.get('lat'), rear_gps.get('lon')
            
            if None not in (lat1, lon1, lat2, lon2):
                # Calculate bearing and distance between the two GPS receivers
                bearing = calculate_bearing(lat1, lon1, lat2, lon2)
                distance = haversine_distance(lat1, lon1, lat2, lon2)
                
                display.append(f"{COLOR_BOLD}DUAL-GPS DATA{COLOR_RESET}")
                display.append(f"Distance between GPS units: {distance:.2f}m")
                display.append(f"Bearing from Front to Rear: {bearing:.1f}°")
                
                # Draw a simple ASCII compass
                display.append(draw_compass(bearing))
    
    return "\n".join(display)

# Other utility functions (haversine_distance, draw_compass, etc.) remain the same

def calculate_bearing(lat1, lon1, lat2, lon2):
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
        return 0.0

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate distance between two points using geopy's geodesic distance"""
    return geodesic((lat1, lon1), (lat2, lon2)).meters

def draw_compass(heading, width=40):
    """Draw more intuitive ASCII compass showing heading"""
    if heading is None:
        return "Compass: No heading data"
        
    # Define compass rose points
    points = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    angles = [0, 45, 90, 135, 180, 225, 270, 315]
    
    # Find closest compass point
    closest_idx = min(range(len(angles)), key=lambda i: min(abs(angles[i] - heading), abs(angles[i] - heading + 360)))
    closest_point = points[closest_idx]
    
    # Create compass visualization
    compass = []
    compass.append(f"Heading: {heading:.1f}° ({closest_point})")
    
    # Create a circular compass visualization
    # First, create a compass rose with 8 directions
    rose = [
        "      N       ",
        "    .   .     ",
        "  .       .   ",
        "W           E ",
        "  .       .   ",
        "    .   .     ",
        "      S       "
    ]
    
    # Calculate where to place the heading indicator
    heading_rad = math.radians((heading + 90) % 360)  # Rotate 90 degrees to match compass orientation
    
    # Calculate x,y coordinates for the indicator on the compass rose
    # Compass is roughly 15x7 characters in size
    center_x = 7
    center_y = 3
    radius = 3
    
    # Convert polar to cartesian coordinates
    x = center_x + round(radius * math.cos(heading_rad))
    y = center_y + round(radius * math.sin(heading_rad))
    
    # Place indicator on the compass
    compass_with_indicator = list(rose)
    if 0 <= y < len(compass_with_indicator) and 0 <= x < len(compass_with_indicator[y]):
        row = list(compass_with_indicator[y])
        row[x] = '●'  # Use a solid circle as the indicator
        compass_with_indicator[y] = ''.join(row)
    
    compass.extend(compass_with_indicator)
    
    # Add legend showing front-rear relationship
    front_pos = "●"  # Front GPS position (center)
    compass.append("")
    compass.append(f"Front GPS at center ({front_pos}), heading to Rear GPS")
    
    return "\n".join(compass)

def create_flask_app(monitor):
    """Create a Flask app for API access"""
    app = Flask(__name__)
    
    @app.route('/api/gps', methods=['GET'])
    def get_gps_data():
        with monitor.lock:
            devices_snapshot = {k: v.copy() for k, v in monitor.devices.items()}
        
        # Convert to JSON structure
        json_data = {
            "timestamp": time.time(),
            "devices": {}
        }
        
        # Sort devices by front/rear
        sorted_devices = sorted(devices_snapshot.items(), 
                             key=lambda x: 0 if x[1].get('name', '').startswith('Front') else 
                                         (1 if x[1].get('name', '').startswith('Rear') else 2))
        
        for port, data in sorted_devices:
            # Clean up data for JSON
            clean_data = {k: v for k, v in data.items() 
                        if k not in ['sentences', 'sentence_counts_snapshot']}
            json_data["devices"][data.get('name', port)] = clean_data
        
        # Add dual-GPS data if available
        if len(devices_snapshot) >= 2:
            front_gps = None
            rear_gps = None
            
            # Find front and rear GPS
            for port, data in devices_snapshot.items():
                if data.get('name') == 'Front GPS':
                    front_gps = data
                elif data.get('name') == 'Rear GPS':
                    rear_gps = data
            
            if front_gps and rear_gps:
                lat1, lon1 = front_gps.get('lat'), front_gps.get('lon')
                lat2, lon2 = rear_gps.get('lat'), rear_gps.get('lon')
                
                if None not in (lat1, lon1, lat2, lon2):
                    bearing = calculate_bearing(lat2, lon2, lat1, lon1)  # Rear to front = vehicle direction
                    distance = haversine_distance(lat1, lon1, lat2, lon2)
                    
                    json_data["dual_gps"] = {
                        "distance": distance,
                        "bearing": bearing,
                        "front_gps": {
                            "lat": lat1,
                            "lon": lon1,
                        },
                        "rear_gps": {
                            "lat": lat2,
                            "lon": lon2,
                        }
                    }
        
        return jsonify(json_data)
    
    @app.route('/api/gps/distance', methods=['GET'])
    def calculate_distance_endpoint():
        """Calculate distance and bearing between two GPS coordinates."""
        try:
            # Get coordinates from query parameters
            lat1 = float(request.args.get('lat1', 0))
            lon1 = float(request.args.get('lon1', 0))
            lat2 = float(request.args.get('lat2', 0))
            lon2 = float(request.args.get('lon2', 0))
            
            # Calculate distance using geodesic distance
            distance = haversine_distance(lat1, lon1, lat2, lon2)
            
            # Calculate bearing using existing function
            bearing = calculate_bearing(lat1, lon1, lat2, lon2)
            
            # Calculate cartesian coordinates (dx, dy)
            # Convert bearing to radians for trig calculations
            bearing_rad = math.radians(bearing)
            
            # Calculate dx (east-west) and dy (north-south)
            # East is positive X, North is positive Y
            dx = distance * math.sin(bearing_rad)
            dy = distance * math.cos(bearing_rad)
            
            # Determine cardinal direction
            cardinal = get_cardinal_direction(bearing)
            
            return jsonify({
                'success': True,
                'points': {
                    'start': {'lat': lat1, 'lon': lon1},
                    'end': {'lat': lat2, 'lon': lon2}
                },
                'distance': {
                    'meters': distance,
                    'feet': distance * 3.28084,
                    'method': 'geodesic'  # Added to indicate we're using geodesic
                },
                'bearing': {
                    'degrees': bearing,
                    'cardinal': cardinal
                },
                'cartesian': {
                    'dx': dx,  # positive = east, negative = west
                    'dy': dy,  # positive = north, negative = south
                    'angle_rad': bearing_rad,
                    'angle_deg': bearing
                }
            })
        except Exception as e:
            return jsonify({
                'success': False,
                'error': str(e)
            }), 400

    @app.route('/api/gps/distance_from_front', methods=['GET'])
    def calculate_distance_from_front():
        """Calculate distance from Front GPS to specified coordinates."""
        try:
            # Get target coordinates from query parameters
            target_lat = float(request.args.get('lat', 0))
            target_lon = float(request.args.get('lon', 0))
            
            # Get current front GPS position
            with monitor.lock:
                devices_snapshot = {k: v.copy() for k, v in monitor.devices.items()}
            
            # Find the front GPS
            front_gps = None
            for port, data in devices_snapshot.items():
                if data.get('name') == 'Front GPS':
                    front_gps = data
                    break
            
            if not front_gps:
                return jsonify({
                    'success': False,
                    'error': 'Front GPS not available'
                }), 404
            
            # Get current position
            current_lat = front_gps.get('lat')
            current_lon = front_gps.get('lon')
            
            if current_lat is None or current_lon is None:
                return jsonify({
                    'success': False,
                    'error': 'Front GPS position not available'
                }), 404
            
            # Calculate distance and bearing
            distance = haversine_distance(current_lat, current_lon, target_lat, target_lon)
            bearing = calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            
            # Calculate cartesian coordinates
            bearing_rad = math.radians(bearing)
            dx = distance * math.sin(bearing_rad)
            dy = distance * math.cos(bearing_rad)
            
            # Get cardinal direction
            cardinal = get_cardinal_direction(bearing)
            
            return jsonify({
                'success': True,
                'points': {
                    'current': {'lat': current_lat, 'lon': current_lon},
                    'target': {'lat': target_lat, 'lon': target_lon}
                },
                'distance': {
                    'meters': distance,
                    'feet': distance * 3.28084,
                    'method': 'geodesic'
                },
                'bearing': {
                    'degrees': bearing,
                    'cardinal': cardinal
                },
                'cartesian': {
                    'dx': dx,  # positive = east, negative = west
                    'dy': dy,  # positive = north, negative = south
                    'angle_rad': bearing_rad,
                    'angle_deg': bearing
                }
            })
            
        except Exception as e:
            return jsonify({
                'success': False,
                'error': str(e)
            }), 400

    return app

# Add this helper function to convert bearing to cardinal direction
def get_cardinal_direction(bearing):
    """Convert bearing in degrees to cardinal direction."""
    dirs = ['N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 
            'S', 'SSW', 'SW', 'WSW', 'W', 'WNW', 'NW', 'NNW']
    ix = round(bearing / (360. / len(dirs)))
    return dirs[ix % len(dirs)]

def parse_args():
    """Parse command-line arguments"""
    parser = argparse.ArgumentParser(description='Direct GPS Monitor')
    parser.add_argument('--json', action='store_true',
                        help='Output JSON format instead of text display')
    parser.add_argument('--no-gui', action='store_true',
                        help='No interactive display, just print data')
    parser.add_argument('--service', action='store_true',
                        help='Run in service mode (non-interactive, automatic port detection)')
    parser.add_argument('--real', action='store_true',
                        help='Run as silent API service on port 5001')
    return parser.parse_args()

def main():
    """Main function"""
    args = parse_args()
    monitor = DirectGpsMonitor()
    
    # Find available ports
    ports = monitor.find_serial_ports()
    
    if not ports:
        print("No serial ports found!")
        return
    
    if args.service or args.real:
        # Auto-detect all ports in real or service mode
        selected_ports = ports
        
        if args.real:
            # Create and start Flask API in real mode
            app = create_flask_app(monitor)
            flask_thread = threading.Thread(
                target=lambda: app.run(host='0.0.0.0', port=5001, debug=False),
                daemon=True
            )
            flask_thread.start()
            print(f"GPS API running at http://localhost:5001/api/gps (silent mode)")
    else:
        # Interactive mode
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
            # In service mode, assign generic names - we'll update later
            monitor.add_device(port, baud_rate, "GPS")
    
    if not monitor.devices:
        print("No GPS devices connected!")
        return
    
    # Main monitoring loop
    try:
        last_screen_clear = 0
        screen_clear_interval = 1.0  # Clear screen once per second
        role_assign_interval = 5.0   # Check device roles every 5 seconds
        last_role_assign = 0
        
        if not args.real:
            print("\nGPS Monitor running. Press Ctrl+C to exit.")
        
        while True:
            # Calculate update rates
            monitor.calculate_update_rates()
            
            # Periodically update device roles based on RTCM detection
            current_time = time.time()
            if current_time - last_role_assign >= role_assign_interval:
                monitor.assign_device_roles()
                last_role_assign = current_time
            
            # In real mode, we don't output to the console, just update the data
            if args.real:
                time.sleep(0.1)
                continue
                
            # Get a snapshot of devices with lock to avoid dict changes during iteration
            with monitor.lock:
                devices_snapshot = {k: v.copy() for k, v in monitor.devices.items()}
            
            # JSON output mode
            if args.json:
                # Convert to JSON structure
                json_data = {
                    "timestamp": time.time(),
                    "devices": {}
                }
                
                for port, data in devices_snapshot.items():
                    # Clean up data for JSON
                    clean_data = {k: v for k, v in data.items() 
                                if k not in ['sentences', 'sentence_counts_snapshot']}
                    json_data["devices"][port] = clean_data
                
                # Add dual-GPS data if available
                # ... rest of the JSON formatting code ...
                
                # Print JSON once per second
                if current_time - last_screen_clear >= screen_clear_interval:
                    print(json.dumps(json_data))
                    last_screen_clear = current_time
                
            # Interactive display mode
            elif not args.no_gui:
                # Clear screen periodically
                if current_time - last_screen_clear >= screen_clear_interval:
                    os.system('cls' if os.name == 'nt' else 'clear')
                    last_screen_clear = current_time
                    
                    # Print header
                    print("=" * 60)
                    print(f"GPS STATUS MONITOR - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                    print("=" * 60)
                    
                    # Show side-by-side display
                    print(draw_side_by_side_display(devices_snapshot))
            
            # Small sleep to avoid CPU overuse
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        if not args.real:
            print("\nExiting...")
    finally:
        monitor.close()

if __name__ == "__main__":
    main()