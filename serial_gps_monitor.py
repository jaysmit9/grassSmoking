#!/usr/bin/env python3
# filepath: /home/jay/projects/smokingGrass/serial_gps_monitor.py
import serial
import time
import math
import os
import sys
import signal
import threading
from datetime import datetime
import csv
import numpy as np
import os.path
import json
import curses  # For arrow key control
from adafruit_servokit import ServoKit

# Configuration
CONFIG = {
    "front_gps_port": "/dev/ttyACM0",
    "rear_gps_port": "/dev/ttyACM1",
    "baud_rate": 115200,
    "update_interval": 0.2,
    "clear_screen": True,
    "timeout": 0.1,
    "waypoints_file": "data/polygon_data.json",
    "max_speed": 0.3,         # Maximum motor speed (0-1)
    "speed_increment": 0.05,  # Speed change per key press
    "turn_factor": 0.7        # How much to slow inside wheel during turns
}

# Global state
running = True
gps_data = {
    "front": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_quality": 0, "last_update": 0,
        "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
    },
    "rear": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_quality": 0, "last_update": 0,
        "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
    }
}

# Motor control state
motor_state = {
    "left_speed": 0.0,
    "right_speed": 0.0,
    "current_speed": 0.0,
    "turning": 0,  # -1=left, 0=straight, 1=right
    "servo_kit": None
}

waypoints = []
first_waypoint = None
last_rate_update = time.time()

# Initialize servo kit
def init_motors():
    try:
        motor_state["servo_kit"] = ServoKit(channels=16)
        motor_state["servo_kit"].continuous_servo[0].throttle = 0  # Right servo
        motor_state["servo_kit"].continuous_servo[1].throttle = 0  # Left servo
        return True
    except Exception as e:
        print(f"Failed to initialize motors: {e}")
        return False

# Motor control functions
def set_motor_speeds(left, right):
    """Set motor speeds, ensuring they're within bounds"""
    try:
        # Ensure speeds are within -1 to 1
        left = max(-CONFIG["max_speed"], min(CONFIG["max_speed"], left))
        right = max(-CONFIG["max_speed"], min(CONFIG["max_speed"], right))
        
        # Update state
        motor_state["left_speed"] = left
        motor_state["right_speed"] = right
        
        # Set actual motor speeds if servo kit is available
        if motor_state["servo_kit"]:
            motor_state["servo_kit"].continuous_servo[1].throttle = left
            motor_state["servo_kit"].continuous_servo[0].throttle = right
        
        return True
    except Exception as e:
        print(f"Error setting motor speeds: {e}")
        return False

def stop_motors():
    """Emergency stop for motors"""
    try:
        set_motor_speeds(0, 0)
        motor_state["current_speed"] = 0
        motor_state["turning"] = 0
        return True
    except Exception as e:
        print(f"Error stopping motors: {e}")
        return False
<<<<<<< HEAD

=======
def save_current_waypoint():
    """Save current GPS position as a waypoint"""
    try:
        # Use front GPS for position
        lat = gps_data["front"]["lat"]
        lon = gps_data["front"]["lon"]
        
        if lat is None or lon is None or gps_data["front"]["fix_quality"] == 0:
            # This will show up in the curses window since we're using print
            print("\nCannot save waypoint: No valid GPS position")
            return False
        
        # Create the waypoint data (match your existing format)
        new_waypoint = {"lat": lat, "lon": lon}
        
        # Define the output file path
        output_file = "polygon_data_proposed.json"
        
        # Check if file exists and load existing data
        waypoints_list = []
        if os.path.exists(output_file):
            try:
                with open(output_file, 'r') as f:
                    existing_data = json.load(f)
                    if isinstance(existing_data, list):
                        waypoints_list = existing_data
            except Exception as e:
                print(f"\nError reading existing waypoints: {e}")
        
        # Add new waypoint
        waypoints_list.append(new_waypoint)
        
        # Save to file
        with open(output_file, 'w') as f:
            json.dump(waypoints_list, f, indent=2)
        
        # Get current time for notification
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Show success notification that will appear in the curses interface
        print(f"\n✅ SAVED WAYPOINT #{len(waypoints_list)} @ {timestamp}")
        print(f"✅ ({lat:.7f}, {lon:.7f}) → {os.path.abspath(output_file)}")
        
        return True
        
    except Exception as e:
        print(f"\nError saving waypoint: {e}")
        return False
    
>>>>>>> gpsd-refactor
def handle_keyboard(key):
    """Handle keyboard input for motor control"""
    current = motor_state["current_speed"]
    turning = motor_state["turning"]
    speed_change = CONFIG["speed_increment"]
    
    # Arrow Up/Down: Change speed
    if key == curses.KEY_UP:
        current += speed_change
        current = min(current, CONFIG["max_speed"])
    elif key == curses.KEY_DOWN:
        current -= speed_change
        current = max(current, -CONFIG["max_speed"])
    # Arrow Left/Right: Turn
    elif key == curses.KEY_LEFT:
        turning = -1
    elif key == curses.KEY_RIGHT:
        turning = 1
    # Space: Stop
    elif key == ord(' '):
        current = 0
        turning = 0
    # 'c' key to center steering
    elif key == ord('c'):
        turning = 0
<<<<<<< HEAD
=======
    # 'p' key to save current position as a waypoint
    elif key == ord('p'):
        save_current_waypoint()
>>>>>>> gpsd-refactor
    
    motor_state["current_speed"] = current
    motor_state["turning"] = turning
    
    # Calculate differential speeds for turning
    left_speed = current
    right_speed = current
    
    # IMPORTANT: Apply minimum turn speed when stationary
    min_turn_speed = CONFIG["max_speed"] * 0.5  # 50% of max speed for turning in place
    
    if turning == -1:  # Turn left
        if current > 0:  # Moving forward
            left_speed = current * (1 - CONFIG["turn_factor"])
        elif current < 0:  # Moving backward
            left_speed = current * (1 + CONFIG["turn_factor"])
        else:  # Stationary - spin in place
            left_speed = -min_turn_speed
            right_speed = min_turn_speed
    elif turning == 1:  # Turn right
        if current > 0:  # Moving forward
            right_speed = current * (1 - CONFIG["turn_factor"])
        elif current < 0:  # Moving backward
            right_speed = current * (1 + CONFIG["turn_factor"])
        else:  # Stationary - spin in place
            left_speed = min_turn_speed
            right_speed = -min_turn_speed
    
    set_motor_speeds(left_speed, right_speed)

def parse_nmea(sentence, gps_position):
    """Parse NMEA sentences and update GPS data"""
    global gps_data
    
    try:
        if not sentence.startswith('$'):
            return
            
        parts = sentence.split(',')
        sentence_type = parts[0]
        
        # Update message counter for rate calculation
        gps_data[gps_position]["message_count"] += 1
        gps_data[gps_position]["last_message_time"] = time.time()
        
        # Parse GGA sentence (position and fix quality)
        if sentence_type == "$GPGGA" or sentence_type == "$GNGGA":
            try:
                # Extract fix quality
                fix_quality = int(parts[6]) if parts[6] else 0
                gps_data[gps_position]["fix_quality"] = fix_quality
                
                # Only update position if we have a fix
                if fix_quality > 0:
                    # Extract latitude
                    if parts[2] and parts[3]:
                        lat_deg = float(parts[2][:2])
                        lat_min = float(parts[2][2:])
                        lat_decimal = lat_deg + (lat_min / 60.0)
                        if parts[3] == 'S':
                            lat_decimal = -lat_decimal
                        gps_data[gps_position]["lat"] = lat_decimal
                    
                    # Extract longitude
                    if parts[4] and parts[5]:
                        lon_deg = float(parts[4][:3])
                        lon_min = float(parts[4][3:])
                        lon_decimal = lon_deg + (lon_min / 60.0)
                        if parts[5] == 'W':
                            lon_decimal = -lon_decimal
                        gps_data[gps_position]["lon"] = lon_decimal
                    
                    gps_data[gps_position]["last_update"] = time.time()
            except (ValueError, IndexError) as e:
                pass
        
        # Parse RMC sentence (position, speed, heading)
        elif sentence_type == "$GPRMC" or sentence_type == "$GNRMC":
            try:
                # Check if data is valid
                if parts[2] == 'A':  # A = valid, V = invalid
                    # Extract speed in knots, convert to m/s
                    if parts[7]:
                        speed_knots = float(parts[7])
                        speed_ms = speed_knots * 0.514444
                        gps_data[gps_position]["speed"] = speed_ms
                    
                    # Extract heading
                    if parts[8]:
                        heading = float(parts[8])
                        gps_data[gps_position]["heading"] = heading
            except (ValueError, IndexError) as e:
                pass
                
        # Parse VTG sentence (heading and speed)
        elif sentence_type == "$GPVTG" or sentence_type == "$GNVTG":
            try:
                # Extract true heading
                if parts[1]:
                    heading = float(parts[1])
                    gps_data[gps_position]["heading"] = heading
                
                # Extract speed (usually in km/h at parts[7])
                if parts[7]:
                    speed_kmh = float(parts[7])
                    speed_ms = speed_kmh / 3.6
                    gps_data[gps_position]["speed"] = speed_ms
            except (ValueError, IndexError) as e:
                pass
        
    except Exception as e:
        print(f"Error parsing NMEA: {e}")

def gps_reader_thread(serial_port, gps_position):
    """Thread to continuously read from the serial port"""
    global running
    
    try:
        ser = serial.Serial(serial_port, CONFIG["baud_rate"], timeout=CONFIG["timeout"])
        buffer = ""
        
        while running:
            try:
                # Read available data
                data = ser.read(ser.in_waiting or 1)
                
                # Process the data
                if data:
                    # Convert bytes to string
                    text = data.decode('ascii', errors='replace')
                    buffer += text
                    
                    # Process any complete sentences in the buffer
                    while '\n' in buffer:
                        # Extract a sentence
                        idx = buffer.find('\n')
                        sentence = buffer[:idx].strip()
                        buffer = buffer[idx+1:]
                        
                        # Parse the sentence
                        if sentence:
                            parse_nmea(sentence, gps_position)
                
                # Small delay to prevent CPU hogging
                time.sleep(0.001)
                
            except UnicodeDecodeError:
                # Handle bad data
                buffer = ""
            except Exception as e:
                print(f"Error in {gps_position} GPS reader: {e}")
                time.sleep(1)  # Avoid rapid error loops
                
    except Exception as e:
        print(f"Could not open {serial_port}: {e}")
    
    finally:
        try:
            ser.close()
        except:
            pass

def haversine_distance(lat1, lon1, lat2, lon2):
    """Calculate the great circle distance between two points in meters"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a))
    r = 6371000  # Radius of earth in meters
    return c * r

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate the bearing between two points in degrees"""
    # Convert decimal degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    # Calculate bearing
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = (math.degrees(bearing) + 360) % 360
    
    return bearing_deg

def calculate_dual_gps_heading():
    """Calculate heading based on positions of front and rear GPS"""
    if (gps_data["front"]["lat"] is None or 
        gps_data["rear"]["lat"] is None or 
        gps_data["front"]["fix_quality"] == 0 or 
        gps_data["rear"]["fix_quality"] == 0):
        return None
    
    # Calculate heading as bearing from rear to front GPS
    heading = calculate_bearing(
        gps_data["rear"]["lat"], gps_data["rear"]["lon"],
        gps_data["front"]["lat"], gps_data["front"]["lon"]
    )
    
    return heading

def clear_screen():
    """Clear the terminal screen"""
    if CONFIG["clear_screen"]:
        os.system('clear' if os.name == 'posix' else 'cls')

def format_position(lat, lon):
    """Format a lat/lon position for display"""
    if lat is None or lon is None:
        return "No position"
    return f"{lat:.7f}, {lon:.7f}"

def signal_handler(sig, frame):
    """Handle Ctrl+C"""
    global running
    print("\nStopping GPS monitor...")
    running = False
    sys.exit(0)

def update_message_rates():
    """Update the messages per second calculation"""
    global gps_data, last_rate_update
    current_time = time.time()
    elapsed = current_time - last_rate_update
    
    if elapsed >= 1.0:  # Update once per second
        for position in ["front", "rear"]:
            count = gps_data[position]["message_count"]
            gps_data[position]["messages_per_sec"] = count / elapsed
            gps_data[position]["message_count"] = 0
        
        last_rate_update = current_time

def load_waypoints():
    """Load waypoints from JSON file"""
    global waypoints, first_waypoint
    
    try:
        # Find the JSON file path
        script_dir = os.path.dirname(os.path.abspath(__file__))
        file_path = os.path.join(script_dir, CONFIG["waypoints_file"])
        
        # For absolute path specified in the config
        if not os.path.exists(file_path):
            # Try the absolute path directly
            if os.path.exists(CONFIG["waypoints_file"]):
                file_path = CONFIG["waypoints_file"]
            else:
                # Try project root folder
                project_root = os.path.dirname(script_dir)
                file_path = os.path.join(project_root, CONFIG["waypoints_file"])
                
        # Check if file exists
        if not os.path.exists(file_path):
            print(f"Waypoints file not found: {file_path}")
            return False
            
        # Read JSON waypoints
        with open(file_path, 'r') as file:
            data = json.load(file)
            waypoints = []
            
            # Process each waypoint in the JSON array
            for point in data:
                if "lat" in point and "lon" in point:
                    lat = float(point["lat"])
                    lon = float(point["lon"])
                    waypoints.append((lat, lon))
        
        if waypoints:
            first_waypoint = waypoints[0]
            print(f"Loaded {len(waypoints)} waypoints from JSON. First: {first_waypoint}")
            return True
        else:
            print("No valid waypoints found in JSON file")
            return False
            
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return False

def calculate_waypoint_metrics(current_lat, current_lon):
    """Calculate distance and heading to first waypoint"""
    if first_waypoint is None or current_lat is None or current_lon is None:
        return None, None
    
    # Calculate distance to first waypoint
    distance = haversine_distance(
        current_lat, current_lon,
        first_waypoint[0], first_waypoint[1]
    )
    
    # Calculate bearing to first waypoint
    heading = calculate_bearing(
        current_lat, current_lon,
        first_waypoint[0], first_waypoint[1]
    )
    
    return distance, heading

def print_gps_status(stdscr=None):
    """Print current GPS status to terminal or curses window"""
    if stdscr:
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        row = 0
    else:
        clear_screen()
    
    update_message_rates()
    
    # Get RTK status indicators
    front_rtk = get_rtk_status(gps_data["front"]["fix_quality"])
    rear_rtk = get_rtk_status(gps_data["rear"]["fix_quality"])
    
    # Calculate distance between GPSs
    distance_between_gps = None
    if (gps_data["front"]["lat"] is not None and 
        gps_data["rear"]["lat"] is not None):
        distance_between_gps = haversine_distance(
            gps_data["front"]["lat"], gps_data["front"]["lon"],
            gps_data["rear"]["lat"], gps_data["rear"]["lon"]
        )
    
    # Calculate heading from GPSs
    heading = calculate_dual_gps_heading()
    
    # Calculate metrics to first waypoint - use front GPS position
    waypoint_distance = None
    waypoint_heading = None
    if gps_data["front"]["lat"] is not None and first_waypoint is not None:
        waypoint_distance, waypoint_heading = calculate_waypoint_metrics(
            gps_data["front"]["lat"], gps_data["front"]["lon"]
        )
    
    # Get individual GPS headings
    front_heading = gps_data["front"]["heading"]
    rear_heading = gps_data["rear"]["heading"]
    
    # Prepare lines to print
    lines = []
    lines.append(f"=== DIRECT SERIAL GPS MONITOR ===  [Arrow keys to drive, Space to stop]  {time.strftime('%H:%M:%S')}")
    lines.append("=" * 60)
    
    # Add motor control status
    turning_indicator = "◄ LEFT" if motor_state["turning"] == -1 else "RIGHT ►" if motor_state["turning"] == 1 else "STRAIGHT"
    direction = "FORWARD" if motor_state["current_speed"] > 0 else "REVERSE" if motor_state["current_speed"] < 0 else "STOPPED"
    
    lines.append(f"MOTORS: {direction} at {abs(motor_state['current_speed']):.2f} - Turning: {turning_indicator}")
    lines.append(f"  Left: {motor_state['left_speed']:.2f}  Right: {motor_state['right_speed']:.2f}")
<<<<<<< HEAD
=======
    lines.append(f"  [Space=STOP] [c=Center] [p=Save Waypoint] [Arrow keys=Drive]")  # Add p key info
>>>>>>> gpsd-refactor
    lines.append("-" * 60)
    
    # Print update rate information
    lines.append(f"Front GPS: {gps_data['front']['messages_per_sec']:.1f} msg/sec | " +
          f"Rear GPS: {gps_data['rear']['messages_per_sec']:.1f} msg/sec")
    
    # Print GPS quality information with RTK status
    lines.append(f"Front GPS: {'FIX' if gps_data['front']['fix_quality'] > 0 else 'NO FIX'} " +
          f"(Quality: {gps_data['front']['fix_quality']}) {front_rtk}" +
          f"    Rear GPS: {'FIX' if gps_data['rear']['fix_quality'] > 0 else 'NO FIX'} " +
          f"(Quality: {gps_data['rear']['fix_quality']}) {rear_rtk}")
    lines.append("-" * 60)
    
    # Print position data
    lines.append(f"Front Position: {format_position(gps_data['front']['lat'], gps_data['front']['lon'])}")
    lines.append(f"Rear Position:  {format_position(gps_data['rear']['lat'], gps_data['rear']['lon'])}")
    lines.append("-" * 60)
    
    # Print heading information
    lines.append(f"Front GPS Heading: {front_heading:.1f}°" if front_heading is not None else "Front GPS Heading: N/A")
    lines.append(f"Rear GPS Heading:  {rear_heading:.1f}°" if rear_heading is not None else "Rear GPS Heading: N/A")
    lines.append(f"Calculated Heading: {heading:.1f}°" if heading is not None else "Calculated Heading: N/A")
    lines.append("-" * 60)
    
    # Print distance information
    lines.append(f"Distance between GPS devices: {distance_between_gps:.2f} meters" if distance_between_gps is not None else "Distance: N/A")
    
    # Print latency information
    front_latency = time.time() - gps_data["front"]["last_update"] if gps_data["front"]["last_update"] > 0 else float('inf')
    rear_latency = time.time() - gps_data["rear"]["last_update"] if gps_data["rear"]["last_update"] > 0 else float('inf')
    lines.append("\nData Freshness:")
    lines.append(f"  Front GPS: {front_latency:.1f} seconds old")
    lines.append(f"  Rear GPS: {rear_latency:.1f} seconds old")
    
    # Print waypoint information
    lines.append("-" * 60)
    lines.append(f"WAYPOINT NAVIGATION:")
    if first_waypoint is not None:
        lines.append(f"First waypoint: {first_waypoint[0]:.7f}, {first_waypoint[1]:.7f}")
        
        if waypoint_distance is not None:
            lines.append(f"Distance to waypoint: {waypoint_distance:.2f} meters")
        else:
            lines.append(f"Distance to waypoint: N/A")
            
        if waypoint_heading is not None:
            lines.append(f"Heading to waypoint: {waypoint_heading:.1f}°")
            
            # Show the heading difference if we have both headings
            if heading is not None:
                heading_diff = (waypoint_heading - heading + 180) % 360 - 180  # Between -180 and 180
                lines.append(f"Heading error: {heading_diff:.1f}° {'(LEFT)' if heading_diff < 0 else '(RIGHT)' if heading_diff > 0 else ''}")
        else:
            lines.append(f"Heading to waypoint: N/A")
    else:
        lines.append(f"No waypoints loaded")
    
    # Print to terminal or curses window
    if stdscr:
        for i, line in enumerate(lines):
            if i < height - 1:  # Avoid writing to bottom line
                stdscr.addstr(i, 0, line[:width-1])
        stdscr.refresh()
    else:
        for line in lines:
            print(line)

def get_rtk_status(fix_quality):
    """Return RTK status indicator based on fix quality"""
    if fix_quality == 4:
        return "[RTK FIXED] 🟢"
    elif fix_quality == 5:
        return "[RTK FLOAT] 🟡"
    elif fix_quality == 2:
        return "[DGPS] 🔵"
    else:
        return ""

def auto_detect_gps_ports():
    """Automatically detect which GPS is front and which is rear based on reported data"""
    print("Auto-detecting GPS positions...")
    
    # Wait for both GPS units to have a fix
    wait_start = time.time()
    while time.time() - wait_start < 10:  # 10 second timeout
        if (gps_data["front"]["fix_quality"] > 0 and 
            gps_data["rear"]["fix_quality"] > 0):
            break
        time.sleep(0.1)
    
    # Check if we have valid fixes
    if (gps_data["front"]["fix_quality"] == 0 or 
        gps_data["rear"]["fix_quality"] == 0):
        print("Could not get valid GPS fixes for auto-detection")
        return False
    
    # Compare GPS positions
    # If front and rear positions are close enough, we can use heading data
    distance_between = haversine_distance(
        gps_data["front"]["lat"], gps_data["front"]["lon"],
        gps_data["rear"]["lat"], gps_data["rear"]["lon"]
    )
    
    print(f"Distance between GPS devices: {distance_between:.2f} meters")
    
    # Get GPS-reported headings
    front_heading = gps_data["front"]["heading"]
    rear_heading = gps_data["rear"]["heading"]
    
    # Calculate expected heading (from rear to front)
    expected_heading = calculate_bearing(
        gps_data["rear"]["lat"], gps_data["rear"]["lon"],
        gps_data["front"]["lat"], gps_data["front"]["lon"]
    )
    
    print(f"Expected heading (rear to front): {expected_heading:.1f}°")
    if front_heading is not None:
        print(f"Front GPS reported heading: {front_heading:.1f}°")
    if rear_heading is not None:
        print(f"Rear GPS reported heading: {rear_heading:.1f}°")
    
    # Decide if ports need to be swapped
    swap_needed = False
    
    # If we have GPS reported headings, use them to verify
    if front_heading is not None and rear_heading is not None:
        # Compare reported headings with calculated heading
        front_heading_error = abs((front_heading - expected_heading + 180) % 360 - 180)
        rear_heading_error = abs((rear_heading - expected_heading + 180) % 360 - 180)
        
        print(f"Front GPS heading error: {front_heading_error:.1f}°")
        print(f"Rear GPS heading error: {rear_heading_error:.1f}°")
        
        if front_heading_error > rear_heading_error:
            swap_needed = True
            print("GPS ports appear to be swapped based on heading data")
    else:
        # Ask user to confirm manually
        print("\nCould not automatically detect GPS positions.")
        response = input("Are the GPS ports swapped? (y/n): ")
        swap_needed = response.lower() == 'y'
    
    if swap_needed:
        print("Swapping front and rear GPS ports...")
        CONFIG["front_gps_port"], CONFIG["rear_gps_port"] = CONFIG["rear_gps_port"], CONFIG["front_gps_port"]
        return True
    else:
        print("GPS ports appear to be correctly configured")
        return False

def curses_main(stdscr):
    """Main function for curses-based arrow key control"""
    global running
    
    # Set up curses
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(100)   # Refresh every 100ms
    
    # Initialize the motors
    init_motors()
    
    try:
        while running:
            # Get input
            key = stdscr.getch()
            
            # Handle input
            if key == ord('q'):
                running = False
                break
            else:
                handle_keyboard(key)
            
            # Update display
            print_gps_status(stdscr)
            time.sleep(CONFIG["update_interval"])
            
    except Exception as e:
        stdscr.clear()
        stdscr.addstr(0, 0, f"Error: {str(e)}")
        stdscr.refresh()
        time.sleep(3)
        
    finally:
        stop_motors()

def main():
    global running
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Load waypoints
    print("Loading waypoints...")
    load_waypoints()
    
    # Start GPS reader threads
    front_thread = threading.Thread(target=gps_reader_thread, 
                                    args=(CONFIG["front_gps_port"], "front"),
                                    daemon=True)
    rear_thread = threading.Thread(target=gps_reader_thread, 
                                    args=(CONFIG["rear_gps_port"], "rear"),
                                    daemon=True)
    
    print("Starting direct serial GPS monitor...")
    front_thread.start()
    rear_thread.start()
    
    # Wait for GPS data to start flowing
    time.sleep(3)
    
    # Auto-detect GPS positions
    if auto_detect_gps_ports():
        # Restart threads with corrected ports
        running = False
        front_thread.join(timeout=2)
        rear_thread.join(timeout=2)
        
        # Reset state
        running = True
        for pos in ["front", "rear"]:
            for key in gps_data[pos]:
                if key not in ["messages_per_sec", "message_count"]:
                    gps_data[pos][key] = None
        
        # Start new threads
        front_thread = threading.Thread(target=gps_reader_thread, 
                                        args=(CONFIG["front_gps_port"], "front"),
                                        daemon=True)
        rear_thread = threading.Thread(target=gps_reader_thread, 
                                        args=(CONFIG["rear_gps_port"], "rear"),
                                        daemon=True)
        front_thread.start()
        rear_thread.start()
        time.sleep(1)
    
    try:
        # Start the curses interface for arrow key control
        curses.wrapper(curses_main)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        running = False
        stop_motors()
        print("GPS monitor stopped")
    
if __name__ == "__main__":
    main()
