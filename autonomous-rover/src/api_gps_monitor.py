#!/usr/bin/env python3

import time
import math
import os
import sys
import signal
import threading
import json
import requests
import curses
from datetime import datetime
import os.path
from adafruit_servokit import ServoKit

# Configuration
CONFIG = {
    "gps_api_url": "http://localhost:5001/api/gps",
    "api_timeout": 0.5,
    "update_interval": 0.2,
    "clear_screen": True,
    "waypoints_file": "data/polygon_data.json",
    "max_speed": 0.3,         # Maximum motor speed (0-1)
    "speed_increment": 0.05,  # Speed change per key press
    "turn_factor": 0.7,       # How much to slow inside wheel during turns
    "waypoint_threshold": 3.0, # Distance in meters to consider waypoint reached
    "waypoint_notification_distance": 10.0  # Distance to start alerting about approaching waypoint
}

# Global state
running = True
gps_data = {
    "front": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_quality": 0, "last_update": 0,
        "satellites_used": 0, "rtk_mode": ""
    },
    "rear": {
        "lat": None, "lon": None, "heading": None, 
        "speed": None, "fix_quality": 0, "last_update": 0,
        "satellites_used": 0, "rtk_mode": ""
    },
    "dual_gps": {
        "bearing": None,
        "distance": None
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

def save_current_waypoint():
    """Save current GPS position as a waypoint"""
    try:
        # Get front GPS position from the API data
        front_gps = gps_data["front"]
        lat = front_gps["lat"]
        lon = front_gps["lon"]
        
        if lat is None or lon is None or front_gps["fix_quality"] == 0:
            print("\nCannot save waypoint: No valid GPS position")
            return False
        
        # Create the waypoint data
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
    # 'p' key to save current position as a waypoint
    elif key == ord('p'):
        save_current_waypoint()
    
    motor_state["current_speed"] = current
    motor_state["turning"] = turning
    
    # Calculate differential speeds for turning
    left_speed = current
    right_speed = current
    
    # Apply minimum turn speed when stationary
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

def get_api_gps_data():
    """Fetch GPS data from API endpoint"""
    try:
        response = requests.get(CONFIG["gps_api_url"], timeout=CONFIG["api_timeout"])
        if response.status_code == 200:
            return response.json()
    except Exception as e:
        print(f"Error fetching GPS data from API: {e}")
    return None

def update_gps_data():
    """Update GPS data from API response"""
    api_data = get_api_gps_data()
    if not api_data:
        return False
    
    try:
        # Extract front GPS data
        front_device = api_data.get('devices', {}).get('Front GPS', {})
        if front_device:
            gps_data["front"]["lat"] = front_device.get('lat')
            gps_data["front"]["lon"] = front_device.get('lon')
            gps_data["front"]["heading"] = front_device.get('track')
            gps_data["front"]["speed"] = front_device.get('speed')
            gps_data["front"]["fix_quality"] = front_device.get('gga_quality', 0)
            gps_data["front"]["last_update"] = front_device.get('last_update', time.time())
            gps_data["front"]["satellites_used"] = front_device.get('satellites_used', 0)
            gps_data["front"]["rtk_mode"] = front_device.get('rtk_mode', '')
        
        # Extract rear GPS data
        rear_device = api_data.get('devices', {}).get('Rear GPS', {})
        if rear_device:
            gps_data["rear"]["lat"] = rear_device.get('lat')
            gps_data["rear"]["lon"] = rear_device.get('lon')
            gps_data["rear"]["heading"] = rear_device.get('track')
            gps_data["rear"]["speed"] = rear_device.get('speed')
            gps_data["rear"]["fix_quality"] = rear_device.get('gga_quality', 0)
            gps_data["rear"]["last_update"] = rear_device.get('last_update', time.time())
            gps_data["rear"]["satellites_used"] = rear_device.get('satellites_used', 0)
            gps_data["rear"]["rtk_mode"] = rear_device.get('rtk_mode', '')
        
        # Extract dual GPS data
        dual_gps = api_data.get('dual_gps', {})
        if dual_gps:
            gps_data["dual_gps"]["bearing"] = dual_gps.get('bearing')
            gps_data["dual_gps"]["distance"] = dual_gps.get('distance')
        
        return True
    except Exception as e:
        print(f"Error processing GPS API data: {e}")
        return False

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
    stop_motors()
    sys.exit(0)

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

def get_rtk_status(fix_quality, rtk_mode):
    """Return RTK status indicator based on fix quality and rtk_mode"""
    if fix_quality == 4 or rtk_mode == "RTK_FIXED":
        return "[RTK FIXED] 🟢"
    elif fix_quality == 5 or rtk_mode == "RTK_FLOAT":
        return "[RTK FLOAT] 🟡"
    elif fix_quality == 2 or rtk_mode == "DGPS" or rtk_mode == "3D_DGPS":
        return "[DGPS] 🔵"
    elif fix_quality == 1 or rtk_mode == "3D_FIX":
        return "[3D FIX] ⚪"
    else:
        return ""

def print_gps_status(stdscr=None):
    """Print current GPS status to terminal or curses window"""
    if stdscr:
        stdscr.clear()
        height, width = stdscr.getmaxyx()
        row = 0
    else:
        clear_screen()
    
    # Get RTK status indicators
    front_rtk = get_rtk_status(gps_data["front"]["fix_quality"], gps_data["front"]["rtk_mode"])
    rear_rtk = get_rtk_status(gps_data["rear"]["fix_quality"], gps_data["rear"]["rtk_mode"])
    
    # Get distance between GPSs from API data
    distance_between_gps = gps_data["dual_gps"]["distance"]
    
    # Get dual GPS heading from API
    heading = gps_data["dual_gps"]["bearing"]
    
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
    lines.append(f"=== API GPS MONITOR ===  [Arrow keys to drive, Space to stop]  {time.strftime('%H:%M:%S')}")
    lines.append("=" * 60)
    
    # Add motor control status
    turning_indicator = "◄ LEFT" if motor_state["turning"] == -1 else "RIGHT ►" if motor_state["turning"] == 1 else "STRAIGHT"
    direction = "FORWARD" if motor_state["current_speed"] > 0 else "REVERSE" if motor_state["current_speed"] < 0 else "STOPPED"
    
    lines.append(f"MOTORS: {direction} at {abs(motor_state['current_speed']):.2f} - Turning: {turning_indicator}")
    lines.append(f"  Left: {motor_state['left_speed']:.2f}  Right: {motor_state['right_speed']:.2f}")
    lines.append(f"  [Space=STOP] [c=Center] [p=Save Waypoint] [Arrow keys=Drive]")
    lines.append("-" * 60)
    
    # Print GPS quality information with RTK status
    lines.append(f"Front GPS: {'FIX' if gps_data['front']['fix_quality'] > 0 else 'NO FIX'} " +
          f"(Quality: {gps_data['front']['fix_quality']}) {front_rtk}" +
          f"    Satellites: {gps_data['front']['satellites_used']}")
    
    lines.append(f"Rear GPS: {'FIX' if gps_data['rear']['fix_quality'] > 0 else 'NO FIX'} " +
          f"(Quality: {gps_data['rear']['fix_quality']}) {rear_rtk}" +
          f"    Satellites: {gps_data['rear']['satellites_used']}")
    
    lines.append("-" * 60)
    
    # Print position data
    lines.append(f"Front Position: {format_position(gps_data['front']['lat'], gps_data['front']['lon'])}")
    lines.append(f"Rear Position:  {format_position(gps_data['rear']['lat'], gps_data['rear']['lon'])}")
    lines.append("-" * 60)
    
    # Print heading information
    lines.append(f"Front GPS Heading: {front_heading:.1f}°" if front_heading is not None else "Front GPS Heading: N/A")
    lines.append(f"Rear GPS Heading:  {rear_heading:.1f}°" if rear_heading is not None else "Rear GPS Heading: N/A")
    lines.append(f"Dual GPS Heading: {heading:.1f}°" if heading is not None else "Dual GPS Heading: N/A")
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
            # Add proximity alerts
            if waypoint_distance <= CONFIG["waypoint_threshold"]:
                lines.append(f"🎯 WAYPOINT REACHED! Distance: {waypoint_distance:.2f} meters")
            elif waypoint_distance <= CONFIG["waypoint_notification_distance"]:
                lines.append(f"⚠️ APPROACHING WAYPOINT: {waypoint_distance:.2f} meters")
            else:
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

def check_waypoint_proximity():
    """Check if the rover is close enough to the current waypoint"""
    global waypoints, first_waypoint
    
    # Need valid GPS data and waypoint
    if first_waypoint is None or gps_data["front"]["lat"] is None:
        return False
    
    waypoint_distance, _ = calculate_waypoint_metrics(
        gps_data["front"]["lat"], gps_data["front"]["lon"]
    )
    
    if waypoint_distance is None:
        return False
        
    # Check if we've reached the waypoint
    if waypoint_distance <= CONFIG["waypoint_threshold"]:
        return True
    
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
    
    # Waypoint notification tracking
    waypoint_reached_notification_time = 0
    
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
            
            # Update GPS data
            update_gps_data()
            
            # Check waypoint proximity
            current_time = time.time()
            if check_waypoint_proximity() and current_time - waypoint_reached_notification_time > 5:
                # Show a notification (could also automatically stop here)
                waypoint_reached_notification_time = current_time
                stdscr.addstr(0, 0, "🎯 WAYPOINT REACHED! ", curses.A_REVERSE)
                stdscr.refresh()
                
                # Optional: Automatically stop when reaching waypoint
                # stop_motors()
            
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
    
    print("Starting API GPS monitor...")
    
    # Verify we can connect to the API
    print("Checking API connection...")
    if update_gps_data():
        print("Successfully connected to GPS API")
    else:
        print("Warning: Could not connect to GPS API. Will continue trying...")
    
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