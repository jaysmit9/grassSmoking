#!/usr/bin/env python3
# filepath: ~/projects/grassSmoking/autonomous-rover/src/realsense/utils/test_detection_service.py

import subprocess
import requests
import json
import time
import sys
import argparse
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import atexit
import signal

def print_color(text, color):
    """Print colored text to console"""
    colors = {
        'red': '\033[91m',
        'green': '\033[92m',
        'yellow': '\033[93m',
        'blue': '\033[94m',
        'reset': '\033[0m'
    }
    print(f"{colors.get(color, '')}{text}{colors['reset']}")

def run_command(command, capture_output=True):
    """Run a system command and return result"""
    print_color(f"Running: {command}", "blue")
    try:
        result = subprocess.run(command, shell=True, text=True, 
                               capture_output=capture_output, check=True)
        return result.stdout if capture_output else None
    except subprocess.CalledProcessError as e:
        print_color(f"Command failed: {e}", "red")
        if e.stderr:
            print_color(e.stderr, "red")
        return None

def fetch_gps_data(url="http://localhost:5001/api/gps"):
    """Fetch GPS data from API endpoint"""
    try:
        response = requests.get(url, timeout=2)
        if response.status_code == 200:
            return response.json()
        else:
            print_color(f"GPS API returned status code {response.status_code}", "red")
            return None
    except requests.exceptions.RequestException as e:
        print_color(f"GPS API request failed: {e}", "red")
        return None

def fetch_detection_data(url="http://localhost:5000/api/detections"):
    """Fetch detection data from RealSense API endpoint"""
    try:
        response = requests.get(url, timeout=2)
        if response.status_code == 200:
            return response.json()
        else:
            print_color(f"Detection API returned status code {response.status_code}", "red")
            return None
    except requests.exceptions.RequestException as e:
        print_color(f"Detection API request failed: {e}", "red")
        return None

def gps_to_meters(lat1, lon1, lat2, lon2):
    """Convert GPS coordinates to meters relative to a reference point"""
    # Earth's radius in meters
    R = 6371000
    
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Calculate differences
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad  # FIXED: Properly calculate longitude difference
    
    # Calculate distances
    x = dlon * R * math.cos((lat1_rad + lat2_rad) / 2)
    y = dlat * R
    
    return x, y

def calculate_destination_point(lat1, lon1, bearing, distance):
    """
    Calculate destination GPS coordinates given a starting point, bearing, and distance.
    
    Args:
        lat1, lon1: Starting coordinates in decimal degrees
        bearing: Bearing in degrees (0=north, 90=east, etc)
        distance: Distance in meters
        
    Returns:
        (lat2, lon2): Destination coordinates in decimal degrees
    """
    # Earth's radius in meters
    R = 6371000
    
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    bearing_rad = math.radians(bearing)
    
    # Calculate new position
    angular_distance = distance / R
    
    lat2_rad = math.asin(
        math.sin(lat1_rad) * math.cos(angular_distance) + 
        math.cos(lat1_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
    )
    
    lon2_rad = lon1_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat1_rad),
        math.cos(angular_distance) - math.sin(lat1_rad) * math.sin(lat2_rad)
    )
    
    # Convert back to degrees
    lat2 = math.degrees(lat2_rad)
    lon2 = math.degrees(lon2_rad)
    
    return lat2, lon2

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
        print_color(f"Error calculating bearing: {e}", "red")
        return 0.0

def create_gps_map():
    """Create and update a map of GPS and detection data"""
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Reference point for calculations
    ref_lat, ref_lon = None, None
    
    # Lists to store points for animation
    front_gps_x, front_gps_y = [], []
    rear_gps_x, rear_gps_y = [], []
    detection_points = []
    
    # Annotations for GPS status
    front_status_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, verticalalignment='top')
    rear_status_text = ax.text(0.02, 0.94, "", transform=ax.transAxes, verticalalignment='top')
    bearing_text = ax.text(0.02, 0.90, "", transform=ax.transAxes, verticalalignment='top')
    detection_count_text = ax.text(0.02, 0.86, "", transform=ax.transAxes, verticalalignment='top')
    object_list_text = ax.text(0.98, 0.98, "", transform=ax.transAxes, 
                          verticalalignment='top', horizontalalignment='right',
                          bbox=dict(boxstyle="round,pad=0.5", fc="white", alpha=0.7))

    def init():
        # Set up the plot
        ax.set_xlim(-5, 5)
        ax.set_ylim(-5, 5)
        ax.set_xlabel('East-West (meters)')
        ax.set_ylabel('North-South (meters)')
        ax.set_title('GPS and Detection Map')
        ax.grid(True)
        
        # Create origin marker
        ax.plot(0, 0, 'ko', markersize=8, label='Origin')
        
        # Create a 10x10 meter box
        square = plt.Rectangle((-5, -5), 10, 10, fill=False, edgecolor='gray', linestyle='--')
        ax.add_patch(square)
        
        # Add compass directions
        ax.text(5.1, 0, 'E', fontsize=12)
        ax.text(-5.1, 0, 'W', fontsize=12)
        ax.text(0, 5.1, 'N', fontsize=12)
        ax.text(0, -5.1, 'S', fontsize=12)
        
        return []
    
    # Modify the update function in create_gps_map to enforce the fixed distance
    def update(frame):
        nonlocal ref_lat, ref_lon
        
        # Clear previous detection points
        for artist in detection_points:
            if artist in ax.get_children():
                artist.remove()
        detection_points.clear()
        
        # Fetch GPS data
        gps_data = fetch_gps_data()
        if gps_data and 'devices' in gps_data:
            front_gps = None
            rear_gps = None
            
            # Find front and rear GPS data
            for name, device in gps_data['devices'].items():
                if name == 'Front GPS':
                    front_gps = device
                elif name == 'Rear GPS':
                    rear_gps = device
            
            # Set reference point if not set
            if ref_lat is None and front_gps and 'lat' in front_gps and 'lon' in front_gps:
                ref_lat = front_gps['lat']
                ref_lon = front_gps['lon']
                print_color(f"Set reference point: {ref_lat:.7f}, {ref_lon:.7f}", "green")
            
            if ref_lat is not None:
                # Plot front GPS
                if front_gps and 'lat' in front_gps and 'lon' in front_gps:
                    x, y = gps_to_meters(ref_lat, ref_lon, front_gps['lat'], front_gps['lon'])
                    
                    if len(front_gps_x) == 0 or (x != front_gps_x[-1] or y != front_gps_y[-1]):
                        front_gps_x.append(x)
                        front_gps_y.append(y)
                    
                    # Plot front GPS point
                    ax.plot(front_gps_x[-1], front_gps_y[-1], 'go', markersize=10, label='Front GPS')
                    
                    # Plot path
                    ax.plot(front_gps_x, front_gps_y, 'g-', alpha=0.5)
                    
                    # Update front GPS status text
                    front_status = f"Front GPS: {front_gps.get('rtk_mode', 'Unknown')} | HDOP: {front_gps.get('hdop', 'N/A')}"
                    front_status_text.set_text(front_status)
                    
                    # Get bearing, either from dual_gps or calculate from separate positions
                    bearing = 0  # Default to North
                    if 'dual_gps' in gps_data and gps_data['dual_gps'].get('bearing') is not None:
                        bearing = gps_data['dual_gps'].get('bearing')
                    elif rear_gps and 'lat' in rear_gps and 'lon' in rear_gps:
                        actual_lat1, actual_lon1 = front_gps['lat'], front_gps['lon']
                        actual_lat2, actual_lon2 = rear_gps['lat'], rear_gps['lon']
                        bearing = calculate_bearing(actual_lat1, actual_lon1, actual_lat2, actual_lon2)
                    
                    # Calculate fixed rear position 0.89 meters behind front GPS
                    FIXED_DISTANCE = 0.89  # meters behind front GPS
                    
                    # Convert bearing to opposite direction (front→rear to rear→front)
                    opposite_bearing = (bearing + 180) % 360
                    
                    # Convert to radians for math calculations
                    opposite_bearing_rad = math.radians(opposite_bearing)
                    
                    # Calculate rear position at fixed distance in opposite direction
                    rear_x = front_gps_x[-1] + FIXED_DISTANCE * math.sin(opposite_bearing_rad)
                    rear_y = front_gps_y[-1] + FIXED_DISTANCE * math.cos(opposite_bearing_rad)
                    
                    # Store the calculated rear position
                    if len(rear_gps_x) == 0 or (rear_x != rear_gps_x[-1] or rear_y != rear_gps_y[-1]):
                        rear_gps_x.append(rear_x)
                        rear_gps_y.append(rear_y)
                    
                    # Plot rear GPS point at the fixed location
                    ax.plot(rear_gps_x[-1], rear_gps_y[-1], 'bo', markersize=10, label='Rear GPS')
                    
                    # Plot path
                    ax.plot(rear_gps_x, rear_gps_y, 'b-', alpha=0.5)
                    
                    # Draw a line connecting front to rear
                    ax.plot([front_gps_x[-1], rear_gps_x[-1]], [front_gps_y[-1], rear_gps_y[-1]], 
                            'k-', linewidth=1, alpha=0.7)
                    
                    # Update rear GPS status text (use actual rear GPS data for status)
                    if rear_gps:
                        rear_status = f"Rear GPS: {rear_gps.get('rtk_mode', 'Unknown')} | HDOP: {rear_gps.get('hdop', 'N/A')}"
                        rear_status_text.set_text(rear_status)
                    
                    # Show bearing and fixed distance
                    bearing_info = f"Bearing: {bearing:.1f}° | Fixed Distance: 0.89m"
                    bearing_text.set_text(bearing_info)
        
        # Fetch detection data and plot on the map
        detection_data = fetch_detection_data()
        object_count = 0

        if detection_data and 'objects' in detection_data:
            object_count = len(detection_data['objects'])
            detection_count_text.set_text(f"Detections: {object_count}")
            
            if front_gps_x and front_gps_y:
                # Get the latest front GPS position as reference for plotting detections
                ref_x, ref_y = front_gps_x[-1], front_gps_y[-1]
                
                # Get the bearing of the rover from dual GPS data
                bearing = 0  # Default to North
                if 'dual_gps' in gps_data and gps_data['dual_gps'].get('bearing') is not None:
                    bearing = gps_data['dual_gps'].get('bearing')
                
                print_color(f"Plotting {object_count} detections with bearing {bearing:.1f}°", "blue")
                
                detected_objects = []
                object_colors = []

                for obj in detection_data['objects']:
                    try:
                        if 'distance' in obj and 'angle' in obj:
                            # Convert from polar (distance, angle) to cartesian
                            # Account for rover's bearing
                            rover_bearing = bearing  # The rover's heading
                            obj_angle = obj['angle']
                            obj_distance = obj['distance']
                            
                            # Calculate absolute bearing from North
                            # (Rover bearing + relative object angle)
                            absolute_bearing = (rover_bearing + obj_angle) % 360
                            
                            # Calculate position relative to the front GPS for map plotting
                            obj_angle_rad = math.radians(absolute_bearing)
                            obj_x = ref_x + obj_distance * math.sin(obj_angle_rad)
                            obj_y = ref_y + obj_distance * math.cos(obj_angle_rad)
                            
                            # Calculate actual GPS coordinates of the object
                            if front_gps and 'lat' in front_gps and 'lon' in front_gps:
                                obj_lat, obj_lon = calculate_destination_point(
                                    front_gps['lat'], front_gps['lon'], 
                                    absolute_bearing, obj_distance
                                )
                                
                                # Store GPS coordinates in the object data
                                obj['gps'] = {
                                    'lat': obj_lat,
                                    'lon': obj_lon
                                }
                                
                                # Print detection details with GPS coordinates
                                print_color(f"Detection at angle {obj['angle']}°, distance {obj_distance}m -> " +
                                           f"pos ({obj_x:.2f}, {obj_y:.2f}) -> GPS: {obj_lat:.7f}, {obj_lon:.7f}", "green")
                            else:
                                print_color(f"Detection at angle {obj['angle']}°, distance {obj_distance}m -> pos ({obj_x:.2f}, {obj_y:.2f})", "green")
                            
                            # Object color based on type with increased visibility
                            if obj.get('type') == 'yolo':
                                plot_color = 'r'  # Red for YOLO
                                text_color = 'red'
                                
                                # Create label with GPS coordinates for YOLO detections
                                if 'label' in obj and 'gps' in obj:
                                    label = f"{obj['label']}\n{obj['gps']['lat']:.7f}\n{obj['gps']['lon']:.7f}"
                                elif 'label' in obj:
                                    label = obj['label']
                                else:
                                    label = "Object"
                            else:
                                plot_color = 'm'  # Magenta for other detections
                                text_color = 'purple'
                                
                                # For non-YOLO objects, use simpler label
                                label = "Object"
                                if 'gps' in obj:
                                    label += f"\n{obj['gps']['lat']:.7f}\n{obj['gps']['lon']:.7f}"

                            # Get size information if available
                            object_size = 0.5  # Default size in meters (50cm diameter)
                            if 'size' in obj:
                                # Check what format the size is provided in
                                print_color(f"Raw size data: {obj['size']}", "blue")
                                
                                # If size is given in centimeters, convert to meters
                                if isinstance(obj['size'], (int, float)):
                                    # Assume size is already in centimeters for safety
                                    object_size = obj['size'] / 100  # Convert cm to meters
                                elif isinstance(obj['size'], (list, tuple)) and len(obj['size']) >= 2:
                                    # Width and height given - use average as diameter in centimeters
                                    object_size = (obj['size'][0] + obj['size'][1]) / 200  # Convert average cm to meters
                                
                                # Add size to label
                                label += f"\nSize: {object_size*100:.0f}cm"
                                
                                # Limit maximum displayed size
                                MAX_DISPLAY_SIZE = 2.0  # 2 meters max display size
                                if object_size > MAX_DISPLAY_SIZE:
                                    print_color(f"Capping display size from {object_size:.2f}m to {MAX_DISPLAY_SIZE:.2f}m", "yellow")
                                    object_size = MAX_DISPLAY_SIZE

                            # Add circular patch showing actual object size
                            circle = plt.Circle((obj_x, obj_y), object_size/2, 
                                               color=plot_color, alpha=0.3, 
                                               fill=True)
                            ax.add_patch(circle)
                            detection_points.append(circle)

                            # Plot center point with smaller marker
                            marker = ax.plot(obj_x, obj_y, 'o', color=plot_color, markersize=6, alpha=0.8, 
                                            markeredgecolor='black', markeredgewidth=1)[0]
                            detection_points.append(marker)

                            # Add label
                            #text = ax.text(obj_x + 0.2, obj_y + 0.2, label, 
                            #              fontsize=8, color=text_color, fontweight='bold',
                            #              backgroundcolor='white', alpha=0.7)
                            #detection_points.append(text)

                            # Add this object to the listing for top-right display
                            if 'label' in obj:
                                object_name = obj['label']
                            elif obj.get('type') == 'yolo':
                                object_name = "YOLO Object"
                            else:
                                object_name = "Object"

                            object_text = f"{object_name}: ({obj_x:.1f}, {obj_y:.1f})"
                            if 'gps' in obj:
                                object_text += f"\n  {obj['gps']['lat']:.7f}, {obj['gps']['lon']:.7f}"
                            
                            detected_objects.append(object_text)
                            object_colors.append(text_color)
                            
                    except Exception as e:
                        print_color(f"Error plotting detection: {e}", "red")
            # Update the object list in the top-right
            if detected_objects:
                # Clear any previous text objects
                object_list_text.set_text("")  # Clear main text object
                
                # Initialize a list to hold individual text objects
                object_list_texts = []
                
                # Create header
                header_text = ax.text(0.98, 0.98, "Detected Objects:", transform=ax.transAxes,
                                    verticalalignment='top', horizontalalignment='right',
                                    fontweight='bold', bbox=dict(boxstyle="round,pad=0.5", 
                                                               fc="white", alpha=0.7))
                detection_points.append(header_text)
                
                # Add individual colored entries
                for i, (obj_text, color) in enumerate(zip(detected_objects, object_colors)):
                    y_pos = 0.95 - (i * 0.04)  # Position each entry below the previous one
                    text_obj = ax.text(0.98, y_pos, f"{i+1}. {obj_text}", transform=ax.transAxes,
                                      verticalalignment='top', horizontalalignment='right',
                                      color=color, fontweight='bold',
                                      bbox=dict(boxstyle="round,pad=0.3", fc="white", alpha=0.7))
                    detection_points.append(text_obj)
            else:
                object_list_text.set_text("No objects detected")

        # Keep the legend updated
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        # Keep front GPS centered in view if we have data
        if front_gps_x and front_gps_y:
            ax.set_xlim(front_gps_x[-1] - 5, front_gps_x[-1] + 5)
            ax.set_ylim(front_gps_y[-1] - 5, front_gps_y[-1] + 5)
        
        # Return all artists that must be redrawn
        artists = [front_status_text, rear_status_text, bearing_text, detection_count_text]
        artists.extend(detection_points)  # Add all detection points to be redrawn
        artists.append(object_list_text)
        return artists
    
    # Create the animation
    ani = FuncAnimation(fig, update, frames=None, init_func=init, 
                        blit=True, interval=200, cache_frame_data=False)
    
    # Show the plot
    plt.tight_layout()
    plt.show()

def check_service_running(url, timeout=1):
    """Check if a service at the given URL is running"""
    try:
        response = requests.get(url, timeout=timeout)
        return response.status_code == 200
    except requests.exceptions.RequestException:
        return False

def start_service(service_name, wait_time=5):
    """Start a systemd service and wait for it to initialize"""
    print_color(f"Starting {service_name} service...", "yellow")
    try:
        subprocess.run(f"sudo systemctl start {service_name}", shell=True, check=True)
        print_color(f"Waiting {wait_time} seconds for {service_name} to initialize...", "yellow")
        time.sleep(wait_time)
        return True
    except subprocess.SubprocessError as e:
        print_color(f"Failed to start {service_name}: {e}", "red")
        return False

def stop_service(service_name):
    """Stop a systemd service"""
    print_color(f"Stopping {service_name} service...", "yellow")
    try:
        subprocess.run(f"sudo systemctl stop {service_name}", shell=True, check=True)
        return True
    except subprocess.SubprocessError as e:
        print_color(f"Failed to stop {service_name}: {e}", "red")
        return False

def parse_args():
    parser = argparse.ArgumentParser(description='GPS and Detection Mapping')
    parser.add_argument('--gps-url', type=str, default='http://localhost:5001/api/gps',
                       help='GPS API endpoint URL (default: http://localhost:5001/api/gps)')
    parser.add_argument('--detection-url', type=str, default='http://localhost:5000/api/detections',
                       help='Detection API endpoint URL (default: http://localhost:5000/api/detections)')
    parser.add_argument('--no-plot', action='store_true',
                       help='Do not show the plot, just print data')
    parser.add_argument('--manage-services', action='store_true', default=True,
                       help='Automatically start and stop required services (default: True)')
    parser.add_argument('--no-manage-services', action='store_false', dest='manage_services',
                       help='Do not manage services automatically')
    return parser.parse_args()

def main():
    args = parse_args()
    
    # Track which services we started (to stop them later)
    started_services = []
    
    # Set up function to stop services on exit
    def cleanup_services():
        for service in started_services:
            stop_service(service)
    
    # Register cleanup function
    atexit.register(cleanup_services)
    
    # Also handle keyboard interrupts
    def signal_handler(sig, frame):
        print_color("\nInterrupted, cleaning up...", "yellow")
        cleanup_services()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Check and start services if needed
    if args.manage_services:
        print_color("Checking required services...", "blue")
        
        # Check and start detection service if needed
        detection_running = check_service_running(args.detection_url)
        if not detection_running:
            if start_service("realsense-detection.service"):
                started_services.append("realsense-detection.service")
        else:
            print_color("Detection service is already running", "green")
        
        # Check GPS service (optional - we may want the user to run this separately)
        gps_running = check_service_running(args.gps_url)
        if not gps_running:
            print_color("GPS service is not running - data will be unavailable", "yellow")
            print_color("To start GPS service: sudo systemctl start gps-monitor.service", "yellow")
    
    # Rest of the script continues...
    if args.no_plot:
        # Just fetch and print data once
        print_color("Fetching GPS data...", "blue")
        gps_data = fetch_gps_data(args.gps_url)
        if gps_data:
            print_color("GPS data:", "green")
            print(json.dumps(gps_data, indent=2))
        
        print_color("\nFetching detection data...", "blue")
        detection_data = fetch_detection_data(args.detection_url)
        if detection_data:
            print_color("Detection data:", "green")
            print(json.dumps(detection_data, indent=2))
    else:
        # Show the interactive map
        print_color("Starting GPS and detection mapping...", "green")
        if started_services:
            print_color("Services started by this script will be stopped on exit", "yellow")
        try:
            create_gps_map()
        finally:
            # Ensure cleanup happens if create_gps_map crashes
            cleanup_services()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())