import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
import traceback
from enum import Enum
import math

# Define obstruction types
class ObstructionType(Enum):
    VERTICAL = 1    # Like trees, poles, buckets
    HORIZONTAL = 2  # Like walls, fences, hedges
    UNKNOWN = 3     # Unclassified obstruction

# Define min and max depths in meters
MIN_DEPTH = 0.3  # Objects closer than 0.3 meters will be ignored
MAX_DEPTH = 5.0  # Objects farther than 5.0 meters will be ignored

# Detection parameters
DETECTION_TIMEOUT = 5.0   # Detections older than this will be removed
EDGE_THRESHOLD = 20       # For edge detection
DEPTH_THRESHOLD = .25    # Minimum depth difference to consider a boundary
MIN_CONTOUR_AREA = 1000   # Minimum contour area to consider

# Parameters from text_distance.py for effective obstacle detection
OBSTACLE_DEPTH_THRESHOLD = 0.3  # Objects must be at least 30cm closer than surroundings
WALL_DEPTH_TOLERANCE = 1.50     # Allow up to 25cm variation for walls
MIN_WALL_SPAN = 4               # Minimum grid cells for wall detection

# Add this parameter:
DEBUG_WALLS = False  # Set to True to see individual wall segments

# Function to calculate angle from center
def calculate_angle(x, width, hfov=66):  # 66° is typical for RealSense
    # Calculate normalized position from center (-1 to 1)
    normalized_pos = (x - width/2) / (width/2)
    # Convert to angle based on horizontal field of view
    angle = normalized_pos * (hfov/2)
    return angle

def initialize_camera():
    """Initialize and start the RealSense camera"""
    print("Initializing RealSense camera...")
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    pipeline_profile = pipeline.start(config)
    
    # Get depth scale for reference
    depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale is: {depth_scale}")
    
    # Create align object to align depth frames to color frames
    align = rs.align(rs.stream.color)
    
    return pipeline, pipeline_profile, align

def load_yolo():
    """Load YOLO model for object detection"""
    print("Loading YOLO model...")
    if os.path.isfile("yolov3-tiny.weights") and os.path.isfile("yolov3-tiny.cfg"):
        print("Using YOLOv3-tiny for better performance")
        net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    else:
        print("Using standard YOLOv3")
        net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    
    # Get output layers
    layer_names = net.getLayerNames()
    output_layers = []
    for i in net.getUnconnectedOutLayers():
        if isinstance(i, (list, tuple)):
            output_layers.append(layer_names[i[0] - 1])
        else:
            output_layers.append(layer_names[i - 1])
    
    # Load class names
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    
    return net, output_layers, classes

def detect_simple_obstacles(depth_frame, width=640, height=480):
    """
    Detect obstacles and walls using only depth data patterns
    Using the effective approach from text_distance.py
    """
    # Get depth data
    depth_data = np.asanyarray(depth_frame.get_data())
    
    # Convert to meters
    depth_meters = depth_data.astype(float) / 1000.0
    
    # Replace zeros (invalid measurements) with NaN
    depth_meters[depth_meters == 0] = np.nan
    
    # Results storage
    obstacles = []
    walls = []
    
    # Create a downsampled depth grid for faster processing
    grid_size = 20  # 20x20 grid cells
    grid_width = width // grid_size
    grid_height = height // grid_size
    
    depth_grid = np.zeros((grid_height, grid_width))
    valid_grid = np.zeros((grid_height, grid_width), dtype=bool)
    
    # Fill the grid with average depth values
    for y in range(grid_height):
        for x in range(grid_width):
            # Get region
            y_start = y * grid_size
            y_end = min((y + 1) * grid_size, height)
            x_start = x * grid_size
            x_end = min((x + 1) * grid_size, width)
            
            region = depth_meters[y_start:y_end, x_start:x_end]
            valid_depths = region[(~np.isnan(region)) & 
                                 (region >= MIN_DEPTH) & 
                                 (region <= MAX_DEPTH)]
            
            if len(valid_depths) > 0:
                depth_grid[y, x] = np.median(valid_depths)
                valid_grid[y, x] = True
    
    # WALL DETECTION
    wall_depth_tolerance = WALL_DEPTH_TOLERANCE
    min_wall_span = MIN_WALL_SPAN
    
    # Find horizontal walls by looking for rows with relatively consistent depths
    for y in range(grid_height):
        row_depths = depth_grid[y, :]
        valid_in_row = valid_grid[y, :]
        
        if np.sum(valid_in_row) < grid_width // 4:  # Need 25% of row to be valid
            continue
            
        valid_depths = row_depths[valid_in_row]
        median_depth = np.median(valid_depths)
        
        # Check for consistent depth across the row (wall-like feature)
        consistent_points = 0
        for x in range(grid_width):
            if valid_grid[y, x] and abs(depth_grid[y, x] - median_depth) < wall_depth_tolerance:
                consistent_points += 1
        
        # If >50% of valid points in row have consistent depth, it's a wall
        if consistent_points > 0.5 * np.sum(valid_in_row) and consistent_points >= min_wall_span:
            # Convert grid coordinates back to image coordinates
            y_center = y * grid_size + grid_size // 2
            wall_height = grid_size * 2  # Make walls thicker for visualization
            wall_width = grid_width * grid_size
            
            # Check if this is close to an existing horizontal wall
            duplicate = False
            for existing_wall in walls:
                if (existing_wall["type"] == "horizontal_wall" and 
                    abs(existing_wall["position"][1] - y_center) < grid_size and
                    abs(existing_wall["distance"] - median_depth) < wall_depth_tolerance):
                    duplicate = True
                    break
            
            if not duplicate:
                # Calculate angle from center
                angle = 0  # Horizontal walls generally face forward
                
                walls.append({
                    "type": "horizontal_wall",
                    "label": "Wall",
                    "position": (width//2, y_center),  # center position
                    "distance": median_depth,
                    "angle": angle,
                    "box": [0, y_center - wall_height//2, wall_width, wall_height],
                    "confidence": 0.8,
                    "obstruction_type": ObstructionType.HORIZONTAL
                })
    
    # Find vertical walls similarly with columns
    for x in range(grid_width):
        col_depths = depth_grid[:, x]
        valid_in_col = valid_grid[:, x]
        
        if np.sum(valid_in_col) < grid_height // 4:  # Need 25% of column to be valid
            continue
            
        valid_depths = col_depths[valid_in_col]
        median_depth = np.median(valid_depths)
        
        # Check for consistent depth down the column
        consistent_points = 0
        for y in range(grid_height):
            if valid_grid[y, x] and abs(depth_grid[y, x] - median_depth) < wall_depth_tolerance:
                consistent_points += 1
        
        # If >50% of valid points in column have relatively consistent depth, call it a wall
        if consistent_points > 0.5 * np.sum(valid_in_col) and consistent_points >= min_wall_span:
            # Convert to image coordinates
            x_center = x * grid_size + grid_size // 2
            wall_height = grid_height * grid_size
            wall_width = grid_size * 2  # Make vertical walls thicker
            
            # Check if this is close to an existing vertical wall
            duplicate = False
            for existing_wall in walls:
                if (existing_wall["type"] == "vertical_wall" and 
                    abs(existing_wall["position"][0] - x_center) < grid_size and
                    abs(existing_wall["distance"] - median_depth) < wall_depth_tolerance):
                    duplicate = True
                    break
                    
            if not duplicate:
                # Calculate angle
                angle = calculate_angle(x_center, width)
                
                walls.append({
                    "type": "vertical_wall",
                    "label": "Wall",
                    "position": (x_center, height//2),  # center position
                    "distance": median_depth,
                    "angle": angle,
                    "box": [x_center - wall_width//2, 0, wall_width, wall_height],
                    "confidence": 0.8,
                    "obstruction_type": ObstructionType.VERTICAL
                })
    
    # OBSTACLE DETECTION using text_distance.py approach
    obstacle_depth_threshold = OBSTACLE_DEPTH_THRESHOLD
    min_neighbors_checked = 3
    
    # First pass: Find depth discontinuities
    for y in range(1, grid_height - 1):
        for x in range(1, grid_width - 1):
            if not valid_grid[y, x]:
                continue
                
            current_depth = depth_grid[y, x]
            
            # Check a wider area around the point
            neighbor_coords = []
            for ny in range(y-2, y+3):
                for nx in range(x-2, x+3):
                    if ny == y and nx == x:
                        continue
                    neighbor_coords.append((ny, nx))
            
            depth_diffs = []
            valid_neighbors = 0
            
            for ny, nx in neighbor_coords:
                if 0 <= ny < grid_height and 0 <= nx < grid_width and valid_grid[ny, nx]:
                    depth_diffs.append(depth_grid[ny, nx] - current_depth)
                    valid_neighbors += 1
            
            if valid_neighbors < min_neighbors_checked:
                continue
                
            # If this point is SIGNIFICANTLY closer than its surroundings
            if np.median(depth_diffs) > obstacle_depth_threshold:
                # Convert to image coordinates
                center_x = x * grid_size + grid_size // 2
                center_y = y * grid_size + grid_size // 2
                
                # Calculate angle from center
                angle = calculate_angle(center_x, width)
                
                # Make obstacles larger for better visibility
                obstacle_size = grid_size * 2.5
                
                # Determine if it's more likely vertical or horizontal based on surrounding depths
                vertical_score = 0
                horizontal_score = 0
                
                # Check left and right neighbors for vertical objects
                for dx in [-1, 1]:
                    if 0 <= x+dx < grid_width and valid_grid[y, x+dx]:
                        if abs(depth_grid[y, x] - depth_grid[y, x+dx]) > 0.1:
                            vertical_score += 1
                
                # Check top and bottom neighbors for horizontal objects
                for dy in [-1, 1]:
                    if 0 <= y+dy < grid_height and valid_grid[y+dy, x]:
                        if abs(depth_grid[y, x] - depth_grid[y+dy, x]) > 0.1:
                            horizontal_score += 1
                
                # Classify based on scores
                if vertical_score > horizontal_score:
                    obstruction_type = ObstructionType.VERTICAL
                    label = "Vertical Object"
                elif horizontal_score > vertical_score:
                    obstruction_type = ObstructionType.HORIZONTAL
                    label = "Horizontal Object"
                else:
                    obstruction_type = ObstructionType.UNKNOWN
                    label = "Object"
                
                obstacles.append({
                    "label": label,
                    "position": (center_x, center_y),
                    "distance": current_depth,
                    "angle": angle,
                    "size": obstacle_size,
                    "confidence": min(1.0, np.median(depth_diffs) / obstacle_depth_threshold),
                    "box": [center_x - obstacle_size//2, center_y - obstacle_size//2, 
                           obstacle_size, obstacle_size],
                    "obstruction_type": obstruction_type
                })
    
    # Second pass: Merge nearby obstacles
    merged_obstacles = []
    remaining = obstacles.copy()
    
    # Sort by confidence (most confident first)
    remaining.sort(key=lambda x: x["confidence"], reverse=True)
    
    while remaining:
        current = remaining.pop(0)
        current_x, current_y = current["position"]
        current_dist = current["distance"]
        
        merged = [current]
        i = 0
        
        # Try to merge with larger distance threshold for improved clustering
        merge_space_threshold = grid_size * 3
        merge_depth_threshold = 0.25
        
        while i < len(remaining):
            next_x, next_y = remaining[i]["position"]
            next_dist = remaining[i]["distance"]
            
            # If obstacles are close in both space and depth, merge them
            if (abs(next_x - current_x) < merge_space_threshold and 
                abs(next_y - current_y) < merge_space_threshold and
                abs(next_dist - current_dist) < merge_depth_threshold):
                
                merged.append(remaining.pop(i))
            else:
                i += 1
        
        # Calculate properties of merged obstacle
        if len(merged) > 0:
            # Weighted average by confidence
            total_weight = sum(o.get("confidence", 1.0) for o in merged)
            avg_x = sum(o["position"][0] * o.get("confidence", 1.0) for o in merged) / total_weight
            avg_y = sum(o["position"][1] * o.get("confidence", 1.0) for o in merged) / total_weight
            avg_dist = sum(o["distance"] * o.get("confidence", 1.0) for o in merged) / total_weight
            
            # Size proportional to number and confidence of merged objects
            size = int(max(40, math.sqrt(len(merged)) * grid_size * 1.5))
            
            # Calculate angle
            angle = calculate_angle(avg_x, width)
            
            # Use most common type
            type_counts = {}
            for o in merged:
                t = o["obstruction_type"]
                type_counts[t] = type_counts.get(t, 0) + 1
            
            common_type = max(type_counts.items(), key=lambda x: x[1])[0]
            
            # Name based on type
            if common_type == ObstructionType.VERTICAL:
                label = "Bucket"  # Assuming vertical objects are often buckets in your scene
            elif common_type == ObstructionType.HORIZONTAL:
                label = "Barrier"
            else:
                label = "Object"
            
            merged_obstacles.append({
                "label": label,
                "position": (int(avg_x), int(avg_y)),
                "distance": avg_dist,
                "angle": angle,
                "size": size,
                "confidence": min(1.0, sum(o.get("confidence", 1.0) for o in merged) / len(merged)),
                "box": [int(avg_x - size//2), int(avg_y - size//2), size, size],
                "obstruction_type": common_type
            })
    
    # Skip wall consolidation and just return all walls and obstacles
    detected_objects = merged_obstacles + walls
    
    # Debug info
    print(f"Detected {len(merged_obstacles)} obstacles and {len(walls)} walls")
    
    return detected_objects

def detect_objects_yolo(color_image, depth_frame, net, output_layers, classes):
    """Detect objects using YOLO and get their distance from depth frame"""
    height, width, _ = color_image.shape
    
    # YOLO detection
    blob = cv2.dnn.blobFromImage(color_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    # Process detections
    class_ids = []
    confidences = []
    boxes = []
    distances = []
    
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            # Only consider confident detections
            if confidence > 0.6:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                # Get distance at center of object
                dist = depth_frame.get_distance(center_x, center_y)
                
                # If center point is invalid, sample multiple points
                if dist == 0:
                    dist_samples = []
                    for dx, dy in [(-5, 0), (5, 0), (0, -5), (0, 5)]:
                        px = min(max(0, center_x + dx), width-1)
                        py = min(max(0, center_y + dy), height-1)
                        d = depth_frame.get_distance(px, py)
                        if d > 0:
                            dist_samples.append(d)
                    
                    if dist_samples:
                        dist = np.median(dist_samples)
                
                # Filter objects by depth range
                if dist > 0 and MIN_DEPTH <= dist <= MAX_DEPTH:
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    distances.append(dist)
    
    # Apply non-max suppression to remove overlapping boxes
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
    
    detected_objects = []
    
    if len(indexes) > 0:
        indexes = indexes.flatten()
        for i in indexes:
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            box = boxes[i]
            distance = distances[i]
            
            # Calculate center position and angle
            center_x = box[0] + box[2] // 2
            center_y = box[1] + box[3] // 2
            angle = calculate_angle(center_x, width)
            
            # Determine if object is an obstruction
            is_obstruction = label in ["bench", "chair", "couch", "potted plant", 
                                      "bed", "dining table", "toilet", "tv", "laptop",
                                      "microwave", "oven", "sink", "refrigerator", 
                                      "book", "clock", "vase", "scissors", "teddy bear",
                                      "hair drier", "toothbrush", "wall", "fence"]
            
            # Determine obstruction type based on aspect ratio
            aspect_ratio = box[3] / box[2] if box[2] > 0 else 999
            
            if aspect_ratio > 1.5:
                obstruction_type = ObstructionType.VERTICAL
            elif aspect_ratio < 0.67:
                obstruction_type = ObstructionType.HORIZONTAL
            else:
                obstruction_type = ObstructionType.UNKNOWN
            
            # Create standardized object info
            detected_objects.append({
                "label": label,
                "position": (center_x, center_y),
                "distance": distance,
                "angle": angle,
                "box": box,
                "confidence": confidence,
                "is_known_object": True,
                "obstruction_type": obstruction_type,
                "is_obstruction": is_obstruction
            })
    
    print(f"Detected {len(detected_objects)} objects using YOLO")
    return detected_objects

def create_top_down_view(width, height, objects, max_distance=5.0, show_walls=False):
    """Create a top-down view visualization of obstacles and objects"""
    # Create a blank top-down view image
    top_view = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Draw reference circles for distance
    center_x, center_y = width // 2, height - 20
    for d in range(1, int(max_distance) + 1):
        radius = int((d / max_distance) * (height - 40))
        cv2.circle(top_view, (center_x, center_y), radius, (50, 50, 50), 1)
        cv2.putText(top_view, f"{d}m", (center_x + 5, center_y - radius), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # Draw direction lines - update angle range to match 66 degree FOV
    for angle in range(-33, 34, 11):  # From -33 to +33 degrees in steps of 11
        rad = math.radians(angle)
        end_x = int(center_x + (height - 40) * math.sin(rad))
        end_y = int(center_y - (height - 40) * math.cos(rad))
        cv2.line(top_view, (center_x, center_y), (end_x, end_y), (50, 50, 50), 1)
        cv2.putText(top_view, f"{angle}°", (end_x + 5, end_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # Draw camera position
    cv2.circle(top_view, (center_x, center_y), 5, (0, 255, 0), -1)
    
    # Draw objects (excluding walls unless show_walls is True)
    for obj in objects:
        # Skip walls if show_walls is False
        if not show_walls and "type" in obj and obj["type"] in ["horizontal_wall", "vertical_wall"]:
            continue
            
        distance = obj.get("distance", 0)
        angle = obj.get("angle", 0)
        label = obj.get("label", "")
        obstruction_type = obj.get("obstruction_type", ObstructionType.UNKNOWN)
        is_known = obj.get("is_known_object", False)
        
        if distance > 0 and distance <= max_distance:
            # Convert to cartesian coordinates
            rad = math.radians(-angle)  # Negative because of screen coordinates
            x = int(center_x + distance * (height - 40) / max_distance * math.sin(rad))
            y = int(center_y - distance * (height - 40) / max_distance * math.cos(rad))
            
            # Draw different markers based on object type
            if is_known:
                # Green for known YOLO objects
                color = (0, 255, 0)
                cv2.circle(top_view, (x, y), 6, color, -1)
            elif "type" in obj and obj["type"] in ["horizontal_wall", "vertical_wall"]:
                # Only draw walls if show_walls is True (this part will only run if show_walls=True)
                # Blue for walls
                color = (255, 0, 0)
                
                if obj["type"] == "horizontal_wall":
                    # Draw horizontal walls as lines
                    cv2.line(top_view, 
                            (int(center_x - width/2), y),
                            (int(center_x + width/2), y),
                            color, 2)
                else:
                    # For vertical walls, draw a shorter line
                    wall_length = 20
                    perp_rad = rad + math.pi/2
                    x1 = int(x + wall_length * math.sin(perp_rad))
                    y1 = int(y - wall_length * math.cos(perp_rad))
                    x2 = int(x - wall_length * math.sin(perp_rad))
                    y2 = int(y - wall_length * math.cos(perp_rad))
                    cv2.line(top_view, (x1, y1), (x2, y2), color, 2)
            else:
                # Red for vertical obstructions, purple for horizontal, white for unknown
                if obstruction_type == ObstructionType.VERTICAL:
                    color = (0, 0, 255)  # Red
                    cv2.circle(top_view, (x, y), 6, color, -1)
                elif obstruction_type == ObstructionType.HORIZONTAL:
                    color = (255, 0, 255)  # Purple
                    cv2.rectangle(top_view, (x-5, y-5), (x+5, y+5), color, -1)
                else:
                    color = (255, 255, 255)  # White
                    cv2.drawMarker(top_view, (x, y), color, cv2.MARKER_DIAMOND, 10)
            
            # Add short label and distance
            short_label = label[:6] if is_known else label[:3]
            cv2.putText(top_view, f"{short_label} {distance:.1f}m", (x+5, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return top_view

def draw_objects(color_image, objects):
    display_image = color_image.copy()
    
    for obj in objects:
        x, y, w, h = obj["box"]
        label = obj["label"]
        distance = obj["distance"]
        
        # Special drawing for walls
        if "type" in obj and obj["type"] in ["horizontal_wall", "vertical_wall"]:
            # Blue for walls
            color = (255, 0, 0)  
            
            # Draw the bounding box for walls with semi-transparent fill
            overlay = display_image.copy()
            cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
            cv2.rectangle(overlay, (x, y), (x + w, y + h), color, -1)
            alpha = 0.3  # Transparency factor
            cv2.addWeighted(overlay, alpha, display_image, 1 - alpha, 0, display_image)
            
            # Add wall label with distance
            wall_text = f"{label}: {distance:.2f}m"
            cv2.putText(display_image, wall_text, (x, y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        else:
            # Regular object drawing (unchanged)
            # Choose color based on object type and distance
            if obj.get("is_known_object", False):
                # Green for known YOLO objects with distance-based intensity
                intensity = max(0, min(255, int(255 * (1 - distance/MAX_DEPTH))))
                color = (0, intensity, 0)
            elif "type" in obj and obj["type"] in ["horizontal_wall", "vertical_wall"]:
                # Blue for walls
                color = (255, 0, 0)
            else:
                # Color based on obstruction type
                if obj.get("obstruction_type", None) == ObstructionType.VERTICAL:
                    color = (0, 0, 255)  # Red for vertical objects
                elif obj.get("obstruction_type", None) == ObstructionType.HORIZONTAL:
                    color = (255, 0, 255)  # Purple for horizontal objects
                else:
                    color = (255, 255, 255)  # White for unknown objects
            
            # Draw the bounding box
            x, y, w, h = obj["box"]
            cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
            
            # Prepare text with object info
            if obj.get("is_known_object", False):
                text = f"{label} {obj['confidence']:.2f} {distance:.2f}m {obj['angle']:.1f}°"
            else:
                text = f"{label} {distance:.2f}m {obj['angle']:.1f}°"
            
            # Add background for text
            text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            cv2.rectangle(display_image, (x, y - text_size[1] - 5), 
                         (x + text_size[0], y), (0, 0, 0), -1)
            
            # Draw text
            cv2.putText(display_image, text, (x, y - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    return display_image

def main():
    global MIN_DEPTH, MAX_DEPTH, WALL_DEPTH_TOLERANCE, OBSTACLE_DEPTH_THRESHOLD
    
    try:
        # Initialize camera
        pipeline, pipeline_profile, align = initialize_camera()
        
        # Load YOLO model
        net, output_layers, classes = load_yolo()
        
        # Track FPS
        fps = 0
        frame_count = 0
        start_time = time.time()
        
        # Detection mode (0: YOLO only, 1: Depth only, 2: Combined)
        detection_mode = 2  # Start with combined mode
        
        # Main loop
        print("Starting detection...")
        while True:
            loop_start = time.time()
            
            # Get aligned frames
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("Missing frames, continuing...")
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(np.asanyarray(depth_frame.get_data()), alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # Get image dimensions
            height, width, _ = color_image.shape
            
            # Detect objects based on mode
            detected_objects = []
            
            # Depth-based detection (modes 1 and 2)
            if detection_mode == 1 or detection_mode == 2:
                print("\n--- Running depth-based detection ---")
                depth_objects = detect_simple_obstacles(depth_frame, width, height)
                
                if detection_mode == 1:  # Depth only
                    detected_objects = depth_objects
                    print(f"Using depth-only mode: {len(detected_objects)} objects detected")
            
            # YOLO-based detection (modes 0 and 2)
            if detection_mode == 0 or detection_mode == 2:
                print("--- Running YOLO detection ---")
                yolo_objects = detect_objects_yolo(color_image, depth_frame, net, output_layers, classes)
                
                if detection_mode == 0:  # YOLO only
                    detected_objects = yolo_objects
                    print(f"Using YOLO-only mode: {len(detected_objects)} objects detected")
            
            # Combine results if in combined mode
            if detection_mode == 2:
                detected_objects = yolo_objects + depth_objects
                print(f"Using combined mode: {len(detected_objects)} total objects detected")
            
            # Draw objects on display image
            display_image = draw_objects(color_image, detected_objects)
            
            # Create top-down view
            top_down_size = min(240, height // 3)
            top_down = create_top_down_view(top_down_size, top_down_size, detected_objects, MAX_DEPTH)
            
            # Position top-down view in bottom-right corner
            display_image[height-top_down_size:height, width-top_down_size:width] = top_down
            
            # Create dashboard at top
            dashboard_height = 60
            dashboard = np.zeros((dashboard_height, width, 3), dtype=np.uint8)
            
            # Calculate FPS
            frame_count += 1
            if frame_count >= 30:
                current_time = time.time()
                fps = frame_count / (current_time - start_time)
                frame_count = 0
                start_time = current_time
            
            # Display FPS and mode
            mode_names = ["YOLO Only", "Depth Only", "Combined"]
            cv2.putText(dashboard, f"FPS: {fps:.1f} | Mode: {mode_names[detection_mode]}", 
                      (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display object counts by type
            object_counts = {}
            for obj in detected_objects:
                label = obj["label"]
                if label in object_counts:
                    object_counts[label] += 1
                else:
                    object_counts[label] = 1
            
            # Format count text
            count_text = " | ".join([f"{k}:{v}" for k, v in 
                                    sorted(object_counts.items(), 
                                          key=lambda x: x[1], 
                                          reverse=True)[:5]])  # Show top 5 objects
            
            cv2.putText(dashboard, count_text[:80], (10, 50), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Combine dashboard with display image
            display_with_dashboard = np.vstack([dashboard, display_image])
            
            # Show the final image
            cv2.namedWindow('Combined Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Combined Detection', display_with_dashboard)
            
            # Show depth map if in depth mode
            if detection_mode == 1:
                cv2.namedWindow('Depth View', cv2.WINDOW_NORMAL)
                cv2.imshow('Depth View', depth_colormap)
            else:
                # Close depth view if not in depth mode
                try:
                    cv2.destroyWindow('Depth View')
                except:
                    pass
            
            # Handle keyboard input
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break
            elif key == ord('m') or key == ord('M'):
                # Cycle through detection modes
                detection_mode = (detection_mode + 1) % 3
                print(f"Switched to mode: {mode_names[detection_mode]}")
            elif key == ord('+') or key == ord('='):
                MAX_DEPTH = min(MAX_DEPTH + 0.5, 10.0)
                print(f"Increased max depth to {MAX_DEPTH}m")
            elif key == ord('-') or key == ord('_'):
                MAX_DEPTH = max(MAX_DEPTH - 0.5, MIN_DEPTH + 0.1)
                print(f"Decreased max depth to {MAX_DEPTH}m")
            elif key == ord('[') or key == ord('{'):
                MIN_DEPTH = max(MIN_DEPTH - 0.1, 0.1)
                print(f"Decreased min depth to {MIN_DEPTH}m")
            elif key == ord(']') or key == ord('}'):
                MIN_DEPTH = min(MIN_DEPTH + 0.1, MAX_DEPTH - 0.1)
                print(f"Increased min depth to {MIN_DEPTH}m")
            elif key == ord('w'):
                WALL_DEPTH_TOLERANCE += 0.05
                print(f"Increased wall depth tolerance to {WALL_DEPTH_TOLERANCE:.2f}m")
            elif key == ord('W'):
                WALL_DEPTH_TOLERANCE = max(0.05, WALL_DEPTH_TOLERANCE - 0.05)
                print(f"Decreased wall depth tolerance to {WALL_DEPTH_TOLERANCE:.2f}m")
            elif key == ord('o'):
                OBSTACLE_DEPTH_THRESHOLD += 0.05
                print(f"Increased obstacle depth threshold to {OBSTACLE_DEPTH_THRESHOLD:.2f}m")
            elif key == ord('O'):
                OBSTACLE_DEPTH_THRESHOLD = max(0.05, OBSTACLE_DEPTH_THRESHOLD - 0.05)
                print(f"Decreased obstacle depth threshold to {OBSTACLE_DEPTH_THRESHOLD:.2f}m")
 
            
            # Display help if 'h' is pressed
            elif key == ord('h') or key == ord('H'):
                print("\n--- Controls ---")
                print("ESC: Exit")
                print("m/M: Switch detection mode")
                print("+/-: Increase/decrease max depth")
                print("[/]: Increase/decrease min depth")
                print("w/W: Increase/decrease wall depth tolerance")
                print("o/O: Increase/decrease obstacle depth threshold")
                print("h/H: Show this help\n")
            
            # Calculate processing time and display debug info
            process_time = time.time() - loop_start
            print(f"Frame processed in {process_time:.3f}s")
            
            # Sleep to maintain reasonable frame rate if processing is very fast
            if process_time < 0.033:  # Cap at ~30fps
                time.sleep(0.033 - process_time)
            
            # In the main detection loop:
            if DEBUG_WALLS:
                # Draw unmerged walls for debugging
                debug_image = color_image.copy()
                for wall in walls:  # original unmerged walls
                    x, y, w, h = wall["box"]
                    cv2.rectangle(debug_image, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    
                cv2.namedWindow('Unmerged Walls', cv2.WINDOW_NORMAL)
                cv2.imshow('Unmerged Walls', debug_image)
    
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    finally:
        # Clean up resources
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Application closed")

if __name__ == "__main__":
    main()