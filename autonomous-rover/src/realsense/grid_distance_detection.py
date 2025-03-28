import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
import traceback
import math
import datetime
import logging
import json
import threading
from pathlib import Path
from enum import Enum
import argparse

# Create a variable to store the latest detection data - define this at the module level
latest_detection_data = None

# Add these variables at the top after initializing frame_counter
frame_counter = 0
last_processed_time = 0
consecutive_timeouts = 0  # Track camera timeouts
last_successful_frame = time.time()  # Track when we last got a frame
MAX_TIMEOUTS = 5  # How many timeouts before attempting recovery

# Import Flask for API
from flask import Flask, jsonify
from flask_cors import CORS

# Create Flask app - no static folder needed for API only
app = Flask(__name__)
CORS(app)  # Allow cross-origin requests

# Shared variable for latest detection data
latest_detection_data = None

# API endpoint to get detection data
@app.route('/api/detections', methods=['GET'])
def get_detections():
    global latest_detection_data
    if latest_detection_data:
        return jsonify(latest_detection_data)
    else:
        return jsonify({"error": "No detection data available yet"})

# That's it - no other routes needed!

# Function to run Flask server in a separate thread
def run_flask_server():
    print(f"Starting API server on port 5000...", file=sys.__stdout__)
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)

# Start the flask server in a thread
threading.Thread(target=run_flask_server, daemon=True).start()

# Function to process objects as JSON data
def process_objects_json(objects_data):
    """Process detected objects as JSON data structure"""
    # Convert any numpy values to Python native types
    def convert_numpy(obj):
        if isinstance(obj, (np.integer, np.floating)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, ObstructionType):
            return obj.name  # Convert enum to string
        return obj
    
    # Clean the data for JSON serialization
    clean_data = []
    for obj in objects_data:
        clean_obj = {}
        for key, value in obj.items():
            # Apply convert_numpy to ALL values, not just specific keys
            clean_obj[key] = convert_numpy(value)
        clean_data.append(clean_obj)
    
    # Create the output structure with frame info
    output = {
        "frame_id": frame_counter if 'frame_counter' in globals() else 0,
        "timestamp": time.time(),
        "objects": clean_data
    }
    
    return output

# Function to get the latest detection data for API access
def get_latest_detection():
    """Return the latest detection data for API access"""
    global latest_detection_data
    return latest_detection_data

# First, ensure logging is properly configured before anything else
def setup_logging():
    """Configure more detailed logging"""
    script_dir = Path(__file__).parent.absolute()
    log_dir = script_dir / "logs"
    log_dir.mkdir(exist_ok=True)
    
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"detection_{timestamp}.log"
    
    # Configure more detailed logging with multiple levels
    logging.basicConfig(
        filename=str(log_file),
        level=logging.INFO,  # Set to DEBUG for even more details
        format='%(asctime)s | %(levelname)s | %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    print(f"All output redirected to: {log_file}", file=sys.__stdout__)
    return log_file

# Initialize logging before anything else
log_file_path = setup_logging()
logging.info("Logging initialized")
print(f"Logging to: {log_file_path}", file=sys.__stdout__)

# Function to print JSON to console and update latest_detection_data
def print_objects_json(objects_data):
    """Process detection data and make it available to API"""
    global latest_detection_data
    
    try:
        # Update the global variable for API access
        latest_detection_data = process_objects_json(objects_data)
        
        # Log the data for debugging
        logging.debug(f"Updated detection data with {len(objects_data)} objects")
        
        # Also print to console in compact form
        print(f"Detected {len(objects_data)} objects for API", file=sys.__stdout__, flush=True)
    except Exception as e:
        print(f"ERROR processing JSON: {str(e)}", file=sys.__stdout__)

# Define obstruction types
class ObstructionType(Enum):
    VERTICAL = 1    # Like trees, poles, buckets
    HORIZONTAL = 2  # Like walls, fences, hedges
    UNKNOWN = 3     # Unclassified obstruction

# Global dictionary to track persistent objects with timestamps
persistent_objects = {}

def calculate_angle(x, width, hfov=66):  # 66° is typical for RealSense
    """Calculate angle from center of image"""
    # Calculate normalized position from center (-1 to 1)
    normalized_pos = (x - width/2) / (width/2)
    # Convert to angle based on horizontal field of view
    angle = normalized_pos * (hfov/2)
    return angle

def create_top_down_view(width, height, objects, max_distance=5.0):
    """Create a top-down view visualization with size-accurate object representation"""
    # Create a blank top-down view image (dark gray background)
    top_view = np.zeros((height, width, 3), dtype=np.uint8)
    top_view[:] = (30, 30, 30)  # Dark gray background
    
    # Draw reference circles for distance
    center_x, center_y = width // 2, height - 20
    for d in range(1, int(max_distance) + 1):
        radius = int((d / max_distance) * (height - 40))
        cv2.circle(top_view, (center_x, center_y), radius, (50, 50, 50), 1)
        cv2.putText(top_view, f"{d}m", (center_x + 5, center_y - radius), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    
    # Draw direction lines
    for angle in range(-33, 34, 11):  # From -33 to +33 degrees in steps of 11
        rad = math.radians(angle)
        end_x = int(center_x + (height - 40) * math.sin(rad))
        end_y = int(center_y - (height - 40) * math.cos(rad))
        cv2.line(top_view, (center_x, center_y), (end_x, end_y), (50, 50, 50), 1)
        if angle != 0:  # Don't clutter with 0° label
            cv2.putText(top_view, f"{angle}°", (end_x - 10, end_y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)
    
    # Draw camera position
    cv2.circle(top_view, (center_x, center_y), 5, (0, 255, 0), -1)
    
    # Draw objects with size-scaled markers
    for obj in objects:
        distance = obj["distance"]
        # Skip if no angle data
        if "angle" not in obj:
            continue
            
        angle = obj["angle"]
        label = obj["label"] if "label" in obj else ""
        group_id = obj.get("group_id", 0)
        is_persistent = obj.get("is_persistent", False)
        is_yolo = obj.get("is_yolo", False)
        
        if distance > 0 and distance <= max_distance:
            # Convert to cartesian coordinates
            rad = math.radians(angle)
            x = int(center_x + distance * (height - 40) / max_distance * math.sin(rad))
            y = int(center_y - distance * (height - 40) / max_distance * math.cos(rad))
            
            # Calculate marker size based on object width/size
            base_size = 4
            
            # For YOLO objects, use bounding box width
            if is_yolo:
                color = (255, 0, 0)  # Blue
                obj_width = obj.get("width", 0)
                # Scale by distance (objects appear smaller when farther)
                width_scaled = obj_width / max(distance * 100, 1)
                marker_size = base_size + min(int(width_scaled), 12)
            # For persistent objects
            elif is_persistent:
                color = (255, 0, 255)  # Purple
                obj_size = obj.get("size", 0)
                marker_size = base_size + min(int(obj_size / 2), 10)
            # For grouped grid objects
            elif group_id > 0:
                np.random.seed(group_id)
                color = (
                    np.random.randint(100, 256),
                    np.random.randint(100, 256),
                    np.random.randint(100, 256)
                )
                obj_size = obj.get("size", 0)
                marker_size = base_size + min(int(obj_size / 2), 8)
            # Default for ungrouped objects
            else:
                color = (255, 255, 255)  # White
                marker_size = base_size
            
            # Draw marker with size-appropriate representation
            cv2.circle(top_view, (x, y), marker_size, color, -1)
            
            # Add short label and distance
            label_text = label if is_yolo else f"{distance:.1f}m"
            cv2.putText(top_view, label_text, (x+5, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # Add legend for object types (rest of legend code unchanged)
    # ...
    
    return top_view

def create_distance_grid_visualization(depth_frame, depth_scale, grid_size=20, highlight_threshold=3.0, yolo_objects=[]):
    """
    Create a visualization showing a grid of minimum distances.
    Adds a semi-transparent colored overlay for cells with objects closer than highlight_threshold.
    Excludes the top 5 rows (sky area), bottom 5 rows (ground area), and leftmost/rightmost 3 columns
    (out of rover path) from highlighting.
    Groups adjacent cells that are close together into distinct objects.
    """
    # Get depth data as numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    
    # Get dimensions
    height, width = depth_image.shape
    
    # Calculate cell dimensions
    cell_height = height // grid_size
    cell_width = width // grid_size
    
    # Create a colorized depth image as base
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
    
    # Create empty overlays
    highlight_overlay = np.zeros_like(depth_colormap)
    
    # Debug variables
    close_cells_count = 0
    
    # Define exclusion zones
    ground_row_cutoff = grid_size - 5  # Exclude bottom 5 rows (ground)
    sky_row_cutoff = 5                # Exclude top 5 rows (sky)
    left_col_cutoff = 3               # Exclude leftmost 3 columns
    right_col_cutoff = grid_size - 3  # Exclude rightmost 3 columns
    
    # Draw horizontal lines to show the sky/ground exclusion areas
    ground_line_y = ground_row_cutoff * cell_height
    sky_line_y = sky_row_cutoff * cell_height
    cv2.line(depth_colormap, (0, ground_line_y), (width, ground_line_y), (255, 255, 0), 2)  # Yellow for ground
    cv2.line(depth_colormap, (0, sky_line_y), (width, sky_line_y), (255, 200, 0), 2)  # Orange for sky
    
    # Draw vertical lines to show the left/right path exclusion areas
    left_line_x = left_col_cutoff * cell_width
    right_line_x = right_col_cutoff * cell_width
    cv2.line(depth_colormap, (left_line_x, 0), (left_line_x, height), (0, 255, 255), 2)  # Cyan for left boundary
    cv2.line(depth_colormap, (right_line_x, 0), (right_line_x, height), (0, 255, 255), 2)  # Cyan for right boundary
    
    # Create a binary grid to track cells with close objects
    close_cell_grid = np.zeros((grid_size, grid_size), dtype=np.uint8)
    
    # First pass: identify all close cells and create the binary grid
    for i in range(grid_size):
        for j in range(grid_size):
            # Calculate cell boundaries
            y_start = i * cell_height
            y_end = (i + 1) * cell_height if i < grid_size - 1 else height
            x_start = j * cell_width
            x_end = (j + 1) * cell_width if j < grid_size - 1 else width
            
            # Extract the cell from depth image
            cell = depth_image[y_start:y_end, x_start:x_end]
            
            # Find minimum non-zero distance
            non_zero_cell = cell[cell > 0]
            if non_zero_cell.size > 0:
                min_dist = np.min(non_zero_cell) * depth_scale  # Convert to meters
                
                # Mark close cells in the grid (excluding all exclusion zones)
                if (min_dist < highlight_threshold and 
                    i >= sky_row_cutoff and i < ground_row_cutoff and
                    j >= left_col_cutoff and j < right_col_cutoff):
                    close_cells_count += 1
                    close_cell_grid[i, j] = 1
    
    # Apply connected component analysis to group adjacent close cells
    num_labels, labels = cv2.connectedComponents(close_cell_grid)
    
    # Count cells in each group
    group_sizes = {}
    for i in range(grid_size):
        for j in range(grid_size):
            if close_cell_grid[i, j] == 1:
                group_id = labels[i, j]
                if group_id not in group_sizes:
                    group_sizes[group_id] = 0
                group_sizes[group_id] += 1
    
    # Generate random colors for each object group (excluding background which is 0)
    colors = []
    for k in range(num_labels):
        if k == 0:  # Background
            colors.append((0, 0, 0))  # Black for background
        else:
            # Generate a bright color (avoid dark colors that blend with background)
            np.random.seed(k)  # Ensure consistent colors
            color = (
                np.random.randint(100, 256),  # Blue component
                np.random.randint(100, 256),  # Green component
                np.random.randint(100, 256)   # Red component
            )
            colors.append(color)
    
    # Dictionary to track the best cell for each group (closest to center)
    group_cells = {}
    image_center_x = width / 2
    image_center_y = height / 2
    
    # Second pass: draw grid and apply colors
    for i in range(grid_size):
        for j in range(grid_size):
            # Calculate cell boundaries
            y_start = i * cell_height
            y_end = (i + 1) * cell_height if i < grid_size - 1 else height
            x_start = j * cell_width
            x_end = (j + 1) * cell_width if j < grid_size - 1 else width
            
            # Extract the cell from depth image
            cell = depth_image[y_start:y_end, x_start:x_end]
            
            # Find minimum non-zero distance
            non_zero_cell = cell[cell > 0]
            if non_zero_cell.size > 0:
                min_dist = np.min(non_zero_cell) * depth_scale  # Convert to meters
                
                # Draw grid cell
                cv2.rectangle(depth_colormap, (x_start, y_start), (x_end, y_end), (255, 255, 255), 1)
                
                # Display the minimum distance in this cell
                text = f"{min_dist:.2f}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.3, 1)[0]
                text_x = x_start + (cell_width - text_size[0]) // 2
                text_y = y_start + (cell_height + text_size[1]) // 2
                
                # Calculate cell center 
                cell_center_x = x_start + cell_width // 2
                cell_center_y = y_start + cell_height // 2
                angle = calculate_angle(cell_center_x, width)
                
                # If this is a close cell, color it according to its group
                if close_cell_grid[i, j] == 1:
                    group_id = labels[i, j]
                    if group_id > 0:  # Skip background (0)
                        color = colors[group_id]
                        
                        # Fill with the group color
                        cv2.rectangle(highlight_overlay, (x_start, y_start), (x_end, y_end), color, -1)
                        
                        # Make text yellow for higher contrast
                        cv2.putText(depth_colormap, text, (text_x, text_y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
                        
                        # Calculate distance to image center
                        dist_to_center = math.sqrt(
                            (cell_center_x - image_center_x)**2 + 
                            (cell_center_y - image_center_y)**2
                        )
                        
                        # If this is the first cell of this group or it's closer to center than previous best
                        if group_id not in group_cells or dist_to_center < group_cells[group_id]["dist_to_center"]:
                            is_large = group_sizes.get(group_id, 0) > 3
                            
                            group_cells[group_id] = {
                                "distance": min_dist,
                                "angle": angle,
                                "group_id": group_id,
                                "label": f"Obj{group_id}",
                                "position": (cell_center_x, cell_center_y),
                                "obstruction_type": ObstructionType.UNKNOWN,
                                "dist_to_center": dist_to_center,
                                "size": group_sizes.get(group_id, 0),
                                "is_large": is_large
                            }
                else:
                    # Regular white text for distant cells or excluded cells
                    cv2.putText(depth_colormap, text, (text_x, text_y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
    
    # Update persistent objects tracking
    current_time = time.time()
    active_large_objects = []
    
    # Add newly detected large objects to persistent_objects
    for obj_id, obj_data in group_cells.items():
        if obj_data["is_large"]:
            # Generate a unique ID based on position and size (approximate)
            # This helps track the same object across frames
            obj_key = f"grid_{int(obj_data['position'][0]/10)}_{int(obj_data['position'][1]/10)}_{int(obj_data['angle'])}"
            
            # Update or add to persistent objects
            persistent_objects[obj_key] = {
                "last_seen": current_time,
                "data": obj_data
            }
            active_large_objects.append(obj_key)
    
    # Create list for top-down view (including persistence)
    top_down_objects = list(group_cells.values())
    
    # Add YOLO objects to top-down view
    for yolo_obj in yolo_objects:
        top_down_objects.append({
            "distance": yolo_obj["distance"],
            "angle": yolo_obj["angle"],
            "label": yolo_obj["label"],
            "is_yolo": True
        })
    
    # Add persistent objects that are still valid (not too old)
    for obj_key, obj_info in list(persistent_objects.items()):
        if current_time - obj_info["last_seen"] > 3.0:
            # Remove objects older than 3 seconds
            persistent_objects.pop(obj_key, None)
        elif obj_key not in active_large_objects:  # Only add if not already active
            # Add persistent object
            persistent_obj = obj_info["data"].copy()
            persistent_obj["is_persistent"] = True
            top_down_objects.append(persistent_obj)
    
    # Print debug info about close cells and objects
    if close_cells_count > 0 and close_cells_count % 10 == 0:  # Only print occasionally
        print(f"Found {close_cells_count} cells with objects closer than {highlight_threshold}m")
        print(f"Grouped into {num_labels-1} distinct objects, {len(persistent_objects)} persistent")
    
    # Apply the highlight overlay with opacity
    alpha = 0.7
    result = cv2.addWeighted(depth_colormap, 1.0, highlight_overlay, alpha, 0)
    
    # Draw legends with dark grey background for better readability
    # First legend - colored groups
    legend_text1 = f"Colored groups: < {highlight_threshold}m"
    text_size1 = cv2.getTextSize(legend_text1, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(result, (5, height - 90), (15 + text_size1[0], height - 90 + text_size1[1] + 5),
                 (50, 50, 50), -1)
    cv2.putText(result, legend_text1, (10, height - 85), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Second legend - ground exclusion
    legend_text2 = "Yellow line: Ground area excluded"
    text_size2 = cv2.getTextSize(legend_text2, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(result, (5, height - 65), (15 + text_size2[0], height - 65 + text_size2[1] + 5),
                 (50, 50, 50), -1)
    cv2.putText(result, legend_text2, (10, height - 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Third legend - sky exclusion
    legend_text3 = "Orange line: Sky area excluded"
    text_size3 = cv2.getTextSize(legend_text3, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(result, (5, height - 40), (15 + text_size3[0], height - 40 + text_size3[1] + 5),
                 (50, 50, 50), -1)
    cv2.putText(result, legend_text3, (10, height - 35), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Fourth legend - path boundaries
    legend_text4 = "Cyan lines: Out of rover path"
    text_size4 = cv2.getTextSize(legend_text4, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.rectangle(result, (5, height - 15), (15 + text_size4[0], height - 15 + text_size4[1] + 5),
                 (50, 50, 50), -1)
    cv2.putText(result, legend_text4, (10, height - 10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # Create larger top-down view
    top_down_size = min(300, height // 2)
    top_down = create_top_down_view(top_down_size, top_down_size, top_down_objects, highlight_threshold)
    
    # Position top-down view in bottom-right corner
    result[height-top_down_size:height, width-top_down_size:width] = top_down
    
    return result, top_down_objects

# Check if the required files exist
required_files = ["yolov3-tiny.weights", "yolov3-tiny.cfg", "coco.names"]
for file in required_files:
    if not os.path.isfile(file):
        print(f"Error: {file} not found in the current directory")
        sys.exit(1)

print("Initializing camera...")
# Configure streams
pipeline = rs.pipeline()
config = rs.config()

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='RealSense object detection with distance grid')
    
    # Create mutually exclusive group (can't use both flags together)
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--sim', action='store_true', 
                          help='Simulation mode: Show visualization windows (default)')
    mode_group.add_argument('--real', action='store_true',
                          help='Real mode: No visualization, only console output')
    
    args = parser.parse_args()
    
    # If neither flag is specified, default to --sim mode
    if not args.sim and not args.real:
        args.sim = True
    
    return args

# Add this right after the imports and logging setup
args = parse_arguments()
show_visualization = args.sim
print(f"Running in {'simulation' if show_visualization else 'real'} mode", file=sys.__stdout__)

try:
    # First add depth stream (this works in simple_depth)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    # Then try adding color stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
    
    # Start streaming
    print("Starting camera pipeline...")
    profile = pipeline.start(config)
    print("Camera pipeline started successfully")
    
    # Get the depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale is: {depth_scale}")
    
    # Create align object - but only use it if we successfully get frames
    align = rs.align(rs.stream.color)
    
    # Load YOLO
    print("Loading YOLO model...")
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    layer_names = net.getLayerNames()
    
    # Handle output layers in a simpler way
    output_layers = []
    for i in net.getUnconnectedOutLayers():
        if isinstance(i, (list, tuple)):
            output_layers.append(layer_names[i[0] - 1])
        else:
            output_layers.append(layer_names[i - 1])
    
    print("Loading class names...")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    print(f"Loaded {len(classes)} classes")
    
    # Add this line before starting the detection loop (around line 567)
    frame_counter = 0

    # Add this near the top of your main code, after initializing frame_counter
    frame_counter = 0
    last_processed_time = 0  # Track when we last processed a frame

    print("Starting detection loop...")
    try:
        while True:  # KEEP ONLY THIS OUTER LOOP
            # Increment counter at the beginning of each loop iteration
            frame_counter += 1
            
            current_time = time.time()
            
            # In real mode, throttle by sleeping longer between frames
            if not show_visualization:
                if current_time - last_processed_time < 0.5:
                    time.sleep(0.1)
                    continue
            
            last_processed_time = current_time
            
            # Check if camera has been failing for too long
            if current_time - last_successful_frame > 30:  # 30 seconds without frames
                print("Camera appears frozen for 30+ seconds. Attempting recovery...", file=sys.__stdout__)
                try:
                    # Stop and restart the pipeline
                    pipeline.stop()
                    time.sleep(1)
                    profile = pipeline.start(config)
                    consecutive_timeouts = 0
                    print("Camera pipeline restarted successfully", file=sys.__stdout__)
                except Exception as e:
                    print(f"Error during recovery: {e}", file=sys.__stdout__)
                    time.sleep(5)  # Wait longer before retry
                    continue
            
            # Wait for frames with proper timeout handling
            try:
                timeout_ms = 1000 if show_visualization else 5000
                frames = pipeline.wait_for_frames(timeout_ms=timeout_ms)
                
                # We got frames successfully, reset timeout counter
                consecutive_timeouts = 0
                last_successful_frame = time.time()
                
            except RuntimeError as e:
                # Handle frame timeout
                consecutive_timeouts += 1
                print(f"Frame timeout #{consecutive_timeouts}: {str(e)}", file=sys.__stdout__)
                
                if consecutive_timeouts >= MAX_TIMEOUTS:
                    print(f"Experienced {consecutive_timeouts} consecutive timeouts, attempting recovery...", file=sys.__stdout__)
                    try:
                        # Stop and restart the pipeline
                        pipeline.stop()
                        time.sleep(1)
                        profile = pipeline.start(config)
                        print("Camera pipeline restarted successfully", file=sys.__stdout__)
                    except Exception as recovery_error:
                        print(f"Recovery failed: {recovery_error}", file=sys.__stdout__)
                    
                    # Reset counter even if recovery failed (to avoid endless restart loop)
                    consecutive_timeouts = 0
                
                # Skip this iteration and try again
                time.sleep(0.5)  # Add a small delay before retry
                continue
            
            if not frames:
                print("No frames received", file=sys.__stdout__)
                time.sleep(0.1)
                continue
            
            # Try to align frames safely
            try:
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
            except Exception as e:
                print(f"Error during frame alignment: {e}")
                # Fall back to unaligned frames
                print("Using unaligned frames instead")
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("Missing frames, continuing...")
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # YOLO object detection
            height, width, channels = color_image.shape
            blob = cv2.dnn.blobFromImage(color_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)
            
            # Process detections
            class_ids = []
            confidences = []
            boxes = []
            yolo_objects = []
            
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Apply non-max suppression
            if boxes:
                try:
                    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
                    # Handle different return types depending on OpenCV version
                    indexes = indexes.flatten() if isinstance(indexes, np.ndarray) else indexes
                    
                    for i in indexes:
                        x, y, w, h = boxes[i]  # Properly unpack all 4 values
                        label = str(classes[class_ids[i]])
                        confidence = confidences[i]
                        
                        # Make sure coordinates are within image bounds
                        center_x = min(max(0, x + w//2), width-1)
                        center_y = min(max(0, y + h//2), height-1)
                        
                        # Get distance - handle potential errors
                        try:
                            dist = depth_frame.get_distance(center_x, center_y)
                        except:
                            dist = 0
                            print("Could not get distance")
                        
                        # Draw bounding box
                        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(color_image, f"{label} {confidence:.2f} {dist:.2f}m", 
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        print(f"Detected: {label} ({confidence:.2f}) at {dist:.2f}m")
                        
                        # Add to YOLO objects list with "is_yolo" flag added
                        yolo_objects.append({
                            "distance": dist,
                            "angle": calculate_angle(center_x, width),
                            "label": label,
                            "is_yolo": True  # Add this flag for consistency
                        })
                except Exception as e:
                    print(f"Error in NMS or drawing: {e}")
            
            # Replace all window creation and display with conditional code

            # At the beginning of the main loop, modify your code to check mode:
            # Process the depth and color frames as before
            # ...
            
            # Only create visualizations if in sim mode (to save CPU)
            if show_visualization:
                # Call grid visualization
                grid_vis, top_down_objects = create_distance_grid_visualization(depth_frame, depth_scale, 
                                            grid_size=20, highlight_threshold=3.0, yolo_objects=yolo_objects)
                
                # Show windows
                try:
                    if grid_vis.shape[0] == color_image.shape[0]:
                        images = np.hstack((color_image, grid_vis))
                    else:
                        h, w, _ = color_image.shape
                        grid_vis_resized = cv2.resize(grid_vis, (int(w * (grid_vis.shape[0] / color_image.shape[0])), h))
                        images = np.hstack((color_image, grid_vis_resized))
                    
                    cv2.namedWindow('RealSense Object Detection', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense Object Detection', images)
                    
                    key = cv2.waitKey(1)
                    if key == 27:  # ESC key
                        print("ESC pressed, exiting...")
                        break
                except Exception as e:
                    print(f"Error displaying visualization: {e}")
            else:
                # In real mode, just process data without any visualization or key checking
                _, top_down_objects = create_distance_grid_visualization(depth_frame, depth_scale,
                                      grid_size=20, highlight_threshold=3.0, yolo_objects=yolo_objects)
                # No waitKey call here!if 
                if cv2.waitKey(1) & 0xFF == 27:
                    print("ESC pressed, exiting...")
                    break
            
            # After YOLO detection
            try:
                if 'yolo_objects' in locals() and yolo_objects:
                    logging.info(f"Detected {len(yolo_objects)} YOLO objects: {[obj.get('label', 'unknown') for obj in yolo_objects]}")
                else:
                    logging.info("No YOLO objects detected in this frame")
            except Exception as e:
                logging.error(f"Error logging YOLO objects: {e}")

            # After processing a frame (use variables that definitely exist):
            try:
                logging.info(f"Processed frame #{frame_counter}")
            except Exception as e:
                logging.error(f"Error in frame logging: {e}")

            # For any important events, use safe logging:
            try:
                # [your event code]
                logging.info("Important event happened")
            except Exception as e:
                logging.error(f"Error processing event: {e}")

            # Define a safe logging function
            def safe_log(level, message):
                """Log safely without crashing if something goes wrong"""
                try:
                    if level == "error":
                        logging.error(message)
                    elif level == "warning":
                        logging.warning(message)
                    else:
                        logging.info(message)
                except Exception as e:
                    print(f"Logging error: {e}", file=sys.__stdout__)

            # Use this safe function instead of direct logging calls
            safe_log("info", f"Processing frame #{frame_counter}")

            # After grid processing
            logging.info(f"Processed grid with {len(top_down_objects) if 'top_down_objects' in locals() else 0} objects")
            
            # For any interesting events
            for obj in yolo_objects:
                distance = obj["distance"]
                if distance < 1.0:  # Very close object
                    logging.warning(f"Close object detected: {obj['label']} at {distance:.2f}m")
            
            # After processing YOLO objects and grid objects, combine and output
            # This remains the same in both modes
            all_objects = []
            
            # Add YOLO objects to the list
            for obj in yolo_objects:
                obj_copy = obj.copy()
                obj_copy["type"] = "yolo"
                all_objects.append(obj_copy)
            
            # Add grid objects to the list
            for obj in top_down_objects:
                obj_copy = obj.copy()
                obj_copy["type"] = "grid"
                all_objects.append(obj_copy)
            
            # Process objects and update global variable for API
            if all_objects:
                processed_data = process_objects_json(all_objects)
                latest_detection_data = processed_data  # Update the variable the API will serve
                
                # Just print summary to console (no double processing)
                print(f"Detected {len(all_objects)} objects for API", file=sys.__stdout__, flush=True)
                
    except KeyboardInterrupt:
        print("Keyboard interrupt, stopping...")
            
    except Exception as e:
        logging.error(f"Failed to process frame {frame_counter}: {str(e)}")
        logging.debug(traceback.format_exc())  # Full stack trace at DEBUG level
        print(f"Setup error: {e}")
        traceback.print_exc()
    
finally:
    # Stop streaming
    logging.info("Stopping pipeline...")
    try:
        pipeline.stop()
    except Exception as e:
        logging.error(f"Error stopping pipeline: {e}")
    
    # Only destroy windows if we created them
    if show_visualization:
        cv2.destroyAllWindows()
    
    logging.info("Pipeline stopped, windows closed")
    
    # Restore standard output/error streams
    sys.stdout = sys.__stdout__
    sys.stderr = sys.__stderr__
    print(f"Session completed. Log file saved to: {log_file_path}")

if __name__ == "__main__":
    # Parse command line arguments
    args = parse_arguments()
    show_visualization = args.sim
    
    # Start the API server in a separate thread
    api_thread = threading.Thread(target=run_flask_server, daemon=True)
    api_thread.start()
    print(f"API server started at http://localhost:5000", file=sys.__stdout__)
    
    # Near the beginning of your script, after setting up logging
    # Start the API server in a separate thread
    api_thread = threading.Thread(target=run_flask_server, daemon=True)
    api_thread.start()
    print(f"API server running at http://localhost:5000/api/detections", file=sys.__stdout__)

    try:
        # Continue with the main detection loop
        # ...existing code...
        pass
    finally:
        # ...existing code...
        pass