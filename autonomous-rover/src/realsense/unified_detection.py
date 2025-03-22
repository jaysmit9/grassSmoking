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

# Set detection expiration time (in seconds)
DETECTION_TIMEOUT = 1.0  # Detections older than this will be removed

# Parameters for obstruction detection
EDGE_THRESHOLD = 20  # Lowered for more sensitive edge detection
DEPTH_THRESHOLD = 0.1  # More relaxed depth discontinuity threshold
MIN_CONTOUR_AREA = 300  # Lower minimum area to detect smaller objects
VERTICAL_RATIO_THRESHOLD = 1.5  # Height/width ratio for vertical classification
HORIZONTAL_RATIO_THRESHOLD = 0.67  # Height/width ratio for horizontal classification
CLUSTER_DISTANCE = 0.10  # Maximum distance between depth points to be considered part of same object
BUCKET_DETECTION_SIZE = (0.3, 0.5)  # Expected size range for a bucket (width, height) in meters
WALL_MIN_WIDTH = 0.5  # Minimum width to consider something a wall (in meters)

# Function to calculate angle from center
def calculate_angle(x, width, hfov=66):  # 66-degree horizontal FOV for RealSense
    # Calculate normalized position from center (-1 to 1)
    normalized_pos = (x - width/2) / (width/2)
    # Convert to angle based on horizontal field of view
    angle = normalized_pos * (hfov/2)
    return angle

# Object to represent a tracked item
class TrackedObject:
    def __init__(self, object_id, label, box, distance, angle, confidence, object_type=ObstructionType.UNKNOWN):
        self.id = object_id
        self.label = label
        self.box = box  # [x, y, w, h]
        self.distance = distance
        self.angle = angle
        self.confidence = confidence
        self.last_seen = time.time()
        self.type = object_type
        self.history = [(distance, angle)]  # Track the object's movement
        self.is_known_object = False  # True if detected by YOLO, False if detected as generic obstruction
        self.color = None
    
    def update(self, box, distance, angle, confidence):
        self.box = box
        self.distance = distance
        self.angle = angle 
        self.confidence = confidence
        self.last_seen = time.time()
        # Add to history if the position changed significantly
        if len(self.history) == 0 or abs(self.history[-1][0] - distance) > 0.1 or abs(self.history[-1][1] - angle) > 2:
            self.history.append((distance, angle))
            # Keep history at reasonable length
            if len(self.history) > 10:
                self.history = self.history[-10:]
    
    def is_stale(self, current_time, timeout=DETECTION_TIMEOUT):
        return (current_time - self.last_seen) > timeout
    
    def get_display_color(self):
        if self.color is not None:
            return self.color
            
        # Choose color based on object type
        if self.is_known_object:
            # Known objects from YOLO detection
            if self.distance == 0:
                return (255, 255, 255)  # White for unknown distance
            elif self.distance < 1.0:
                return (0, 0, 255)      # Red for close objects
            elif self.distance < 2.0:
                return (0, 255, 255)    # Yellow for medium distance
            else:
                return (0, 255, 0)      # Green for far objects
        else:
            # Generic obstructions
            if self.type == ObstructionType.VERTICAL:
                return (0, 0, 255)      # Red for vertical obstructions
            elif self.type == ObstructionType.HORIZONTAL:
                return (255, 0, 0)      # Blue for horizontal obstructions
            else:
                return (255, 0, 255)    # Purple for unknown obstructions

# Function to load Yolo model
def load_yolo_model():
    print("Loading YOLO model...")
    if os.path.isfile("yolov3-tiny.weights") and os.path.isfile("yolov3-tiny.cfg"):
        print("Using YOLOv3-tiny for better performance")
        net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    else:
        print("Using standard YOLOv3")
        net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    
    print("Using CPU backend for YOLO")
    
    # Get output layers
    layer_names = net.getLayerNames()
    output_layers = []
    for i in net.getUnconnectedOutLayers():
        if isinstance(i, (list, tuple)):
            output_layers.append(layer_names[i[0] - 1])
        else:
            output_layers.append(layer_names[i - 1])
    
    # Load class names
    print("Loading class names...")
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    print(f"Loaded {len(classes)} classes")
    
    return net, output_layers, classes

# Function to detect objects using YOLO
def detect_objects_yolo(color_image, depth_frame, net, output_layers, classes):
    height, width, channels = color_image.shape
    
    # Use smaller input size for better performance
    blob = cv2.dnn.blobFromImage(color_image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)
    
    # Process detections
    class_ids = []
    confidences = []
    boxes = []
    distances = []
    angles = []
    
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            # Higher confidence threshold for better performance
            if confidence > 0.6:  
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                
                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                
                # Get distance
                dist = depth_frame.get_distance(center_x, center_y)
                
                # If center point has no valid depth, sample multiple points
                if dist == 0:
                    dist_samples = []
                    for dx, dy in [(-5, -5), (5, 5), (-5, 5), (5, -5)]:
                        px = min(max(0, center_x + dx), width-1)
                        py = min(max(0, center_y + dy), height-1)
                        d = depth_frame.get_distance(px, py)
                        if d > 0:
                            dist_samples.append(d)
                    
                    if dist_samples:
                        dist = np.median(dist_samples)
                
                # Calculate angle
                angle = calculate_angle(center_x, width)
                
                # Filter objects based on distance
                if dist > 0 and MIN_DEPTH <= dist <= MAX_DEPTH:
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)
                    distances.append(dist)
                    angles.append(angle)
    
    # Apply non-max suppression
    indices = []
    if boxes:
        try:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.6, 0.4)
            indices = indices.flatten() if isinstance(indices, np.ndarray) else indices
        except:
            pass
    
    detected_objects = []
    for i in indices:
        detected_objects.append({
            "label": str(classes[class_ids[i]]),
            "box": boxes[i],
            "confidence": confidences[i],
            "distance": distances[i],
            "angle": angles[i],
            "is_known": True
        })
    
    return detected_objects

# Function to detect obstructions from depth data
def detect_obstructions(depth_frame, color_image):
    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    height, width = depth_image.shape
    
    # Add debug info
    print("Starting depth-based obstruction detection")
    
    # Create normalized depth image for display
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    
    # Direct sampling approach from text_distance.py
    # This approach was successful in detecting walls and buckets
    detected_obstructions = []
    
    # Create a downsampled grid for efficiency - sample every 20 pixels
    grid_step = 20
    
    # Create debug image for visualization
    edge_display = cv2.cvtColor(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLOR_GRAY2BGR)
    
    # Using a direct sampling approach to find depth discontinuities
    print("Scanning for depth discontinuities...")
    for x_start in range(0, width, grid_step):
        for y_start in range(0, height, grid_step):
            # Measure depth at this point
            center_dist = depth_frame.get_distance(x_start, y_start)
            
            # Skip invalid or out-of-range depths
            if center_dist < MIN_DEPTH or center_dist > MAX_DEPTH:
                continue
                
            # Sample surrounding points
            surrounding_dists = []
            for dx, dy in [(-grid_step, 0), (grid_step, 0), (0, -grid_step), (0, grid_step)]:
                x_sample = min(max(0, x_start + dx), width-1)
                y_sample = min(max(0, y_start + dy), height-1)
                dist = depth_frame.get_distance(x_sample, y_sample)
                if MIN_DEPTH <= dist <= MAX_DEPTH:
                    surrounding_dists.append(dist)
            
            # Must have enough samples
            if len(surrounding_dists) < 2:
                continue
                
            # Check for sharp depth discontinuity
            max_diff = max([abs(center_dist - d) for d in surrounding_dists])
            if max_diff > DEPTH_THRESHOLD:
                # Found potential edge of object
                cv2.circle(edge_display, (x_start, y_start), 3, (0, 0, 255), -1)
                
                # Expand to find the object boundaries
                min_x, min_y = x_start, y_start
                max_x, max_y = x_start, y_start
                
                # Flood fill approach to find connected region with similar depth
                stack = [(x_start, y_start)]
                visited = set([(x_start, y_start)])
                depth_samples = [center_dist]
                
                while stack and len(visited) < 500:  # Limit search to prevent runaway
                    x, y = stack.pop()
                    
                    # Update bounding box
                    min_x = min(min_x, x)
                    min_y = min(min_y, y)
                    max_x = max(max_x, x)
                    max_y = max(max_y, y)
                    
                    # Search neighbors
                    for dx, dy in [(-grid_step, 0), (grid_step, 0), (0, -grid_step), (0, grid_step)]:
                        nx, ny = x + dx, y + dy
                        
                        if (nx, ny) in visited or nx < 0 or ny < 0 or nx >= width or ny >= height:
                            continue
                            
                        # Check if this point is part of the same object
                        # (similar depth, within threshold)
                        point_dist = depth_frame.get_distance(nx, ny)
                        if abs(point_dist - center_dist) < CLUSTER_DISTANCE and MIN_DEPTH <= point_dist <= MAX_DEPTH:
                            stack.append((nx, ny))
                            visited.add((nx, ny))
                            depth_samples.append(point_dist)
                            # Mark visited points on the edge display
                            cv2.circle(edge_display, (nx, ny), 2, (255, 0, 255), -1)
                
                # Only consider if we found enough connected points
                if len(visited) > 10:
                    # Create bounding box with some margin
                    box_x = max(0, min_x - grid_step)
                    box_y = max(0, min_y - grid_step)
                    box_w = min(width - box_x, max_x - min_x + 2*grid_step)
                    box_h = min(height - box_y, max_y - min_y + 2*grid_step)
                    
                    # Calculate center
                    center_x = box_x + box_w // 2
                    center_y = box_y + box_h // 2
                    
                    # Use median of collected depth samples for better accuracy
                    median_dist = np.median(depth_samples)
                    
                    # Calculate aspect ratio to classify the obstruction
                    aspect_ratio = box_h / box_w if box_w > 0 else 999
                    
                    # Determine obstruction type based on aspect ratio
                    if aspect_ratio > VERTICAL_RATIO_THRESHOLD:
                        obstruction_type = ObstructionType.VERTICAL
                        label = "Vertical"
                    elif aspect_ratio < HORIZONTAL_RATIO_THRESHOLD:
                        obstruction_type = ObstructionType.HORIZONTAL
                        label = "Wall"
                    else:
                        obstruction_type = ObstructionType.UNKNOWN
                        label = "Object"
                    
                    # Calculate angle from center
                    angle = calculate_angle(center_x, width)
                    
                    # Add this object if it's not too small
                    if box_w * box_h > MIN_CONTOUR_AREA:
                        print(f"Found {label} at {median_dist:.2f}m, angle: {angle:.1f}°, size: {box_w}x{box_h}")
                        
                        # Draw bounding box on the edge display
                        cv2.rectangle(edge_display, (box_x, box_y), (box_x + box_w, box_y + box_h), 
                                     (0, 255, 0), 2)
                        
                        # Add to detected obstructions
                        detected_obstructions.append({
                            "label": label,
                            "box": [box_x, box_y, box_w, box_h],
                            "confidence": 0.7,  # Default confidence for depth-based detection
                            "distance": median_dist,
                            "angle": angle,
                            "is_known": False,
                            "type": obstruction_type
                        })
    
    print(f"Detected {len(detected_obstructions)} obstructions from depth data")
    return detected_obstructions, depth_colormap, edge_display

# Function to create a top-down view of objects
def create_top_down_view(width, height, tracked_objects, max_distance=5.0):
    # Create a blank top-down view image
    top_view = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Draw reference circles for distance
    center_x, center_y = width // 2, height - 20
    for d in range(1, int(max_distance) + 1):
        radius = int((d / max_distance) * (height - 40))
        cv2.circle(top_view, (center_x, center_y), radius, (50, 50, 50), 1)
        cv2.putText(top_view, f"{d}m", (center_x + 5, center_y - radius), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # Draw direction lines
    for angle in range(-60, 61, 30):
        rad = math.radians(angle)
        end_x = int(center_x + (height - 40) * math.sin(rad))
        end_y = int(center_y - (height - 40) * math.cos(rad))
        cv2.line(top_view, (center_x, center_y), (end_x, end_y), (50, 50, 50), 1)
        cv2.putText(top_view, f"{angle}°", (end_x + 5, end_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
    
    # Draw camera position
    cv2.circle(top_view, (center_x, center_y), 5, (0, 255, 0), -1)
    
    # Draw objects
    for obj in tracked_objects:
        if obj.distance > 0 and obj.distance <= max_distance:
            # Convert to cartesian coordinates
            rad = math.radians(-obj.angle)  # Negative because screen coordinates
            x = int(center_x + obj.distance * (height - 40) / max_distance * math.sin(rad))
            y = int(center_y - obj.distance * (height - 40) / max_distance * math.cos(rad))
            
            # Draw marker based on object type
            color = obj.get_display_color()
            
            if obj.is_known_object:
                # Known object (from YOLO)
                cv2.circle(top_view, (x, y), 7, color, -1)
            else:
                # Obstruction
                if obj.type == ObstructionType.VERTICAL:
                    cv2.circle(top_view, (x, y), 7, color, -1)
                elif obj.type == ObstructionType.HORIZONTAL:
                    # Draw a small line perpendicular to the angle
                    perp_rad = rad + math.pi/2
                    line_length = 15
                    x1 = int(x + line_length * math.sin(perp_rad))
                    y1 = int(y - line_length * math.cos(perp_rad))
                    x2 = int(x - line_length * math.sin(perp_rad))
                    y2 = int(y - line_length * math.cos(perp_rad))
                    cv2.line(top_view, (x1, y1), (x2, y2), color, 2)
                else:
                    cv2.rectangle(top_view, (x-5, y-5), (x+5, y+5), color, -1)
            
            # Draw object label
            label = obj.label[:5]  # Truncate long labels
            cv2.putText(top_view, f"{label} {obj.distance:.1f}m", (x+5, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # Draw history trail if available
            if len(obj.history) > 1:
                for i in range(1, len(obj.history)):
                    prev_dist, prev_angle = obj.history[i-1]
                    curr_dist, curr_angle = obj.history[i]
                    
                    prev_rad = math.radians(-prev_angle)
                    curr_rad = math.radians(-curr_angle)
                    
                    px = int(center_x + prev_dist * (height - 40) / max_distance * math.sin(prev_rad))
                    py = int(center_y - prev_dist * (height - 40) / max_distance * math.cos(prev_rad))
                    cx = int(center_x + curr_dist * (height - 40) / max_distance * math.sin(curr_rad))
                    cy = int(center_y - curr_dist * (height - 40) / max_distance * math.cos(curr_rad))
                    
                    cv2.line(top_view, (px, py), (cx, cy), (100, 100, 100), 1)
    
    return top_view

# Function to update tracked objects
def update_tracked_objects(tracked_objects, detections, current_time, max_iou_distance=0.5):
    # Remove stale tracked objects
    tracked_objects = [obj for obj in tracked_objects if not obj.is_stale(current_time)]
    
    # Map existing tracked objects to new detections
    matched_indices = []
    
    for i, detection in enumerate(detections):
        best_match_idx = -1
        best_match_score = max_iou_distance
        det_box = detection["box"]
        det_label = detection["label"]
        
        for j, tracked_obj in enumerate(tracked_objects):
            # First priority: match by label if it's a known object
            if detection["is_known"] and tracked_obj.is_known_object and det_label == tracked_obj.label:
                # Calculate IoU between boxes
                box1 = det_box
                box2 = tracked_obj.box
                
                # Calculate intersection area
                x1 = max(box1[0], box2[0])
                y1 = max(box1[1], box2[1])
                x2 = min(box1[0] + box1[2], box2[0] + box2[2])
                y2 = min(box1[1] + box1[3], box2[1] + box2[3])
                
                if x2 < x1 or y2 < y1:
                    continue  # No overlap
                
                intersection_area = (x2 - x1) * (y2 - y1)
                box1_area = box1[2] * box1[3]
                box2_area = box2[2] * box2[3]
                union_area = box1_area + box2_area - intersection_area
                
                iou = intersection_area / union_area if union_area > 0 else 0
                
                if iou > best_match_score:
                    best_match_score = iou
                    best_match_idx = j
            
            # Second priority: match by position for generic obstructions
            elif (not detection["is_known"]) and (not tracked_obj.is_known_object):
                # Use distance and angle to match obstructions
                if (abs(detection["distance"] - tracked_obj.distance) < 0.3 and 
                    abs(detection["angle"] - tracked_obj.angle) < 10):
                    # Calculate IoU as above for additional matching criteria
                    box1 = det_box
                    box2 = tracked_obj.box
                    
                    x1 = max(box1[0], box2[0])
                    y1 = max(box1[1], box2[1])
                    x2 = min(box1[0] + box1[2], box2[0] + box2[2])
                    y2 = min(box1[1] + box1[3], box2[1] + box2[3])
                    
                    if x2 < x1 or y2 < y1:
                        continue  # No overlap
                    
                    intersection_area = (x2 - x1) * (y2 - y1)
                    box1_area = box1[2] * box1[3]
                    box2_area = box2[2] * box2[3]
                    union_area = box1_area + box2_area - intersection_area
                    
                    iou = intersection_area / union_area if union_area > 0 else 0
                    
                    if iou > best_match_score:
                        best_match_score = iou
                        best_match_idx = j
        
        # Update the matched tracked object
        if best_match_idx >= 0:
            tracked_objects[best_match_idx].update(
                detection["box"],
                detection["distance"],
                detection["angle"],
                detection["confidence"]
            )
            matched_indices.append(i)
    
    # Add new objects for unmatched detections
    for i, detection in enumerate(detections):
        if i not in matched_indices:
            obj_type = detection.get("type", ObstructionType.UNKNOWN)
            new_obj = TrackedObject(
                len(tracked_objects),
                detection["label"],
                detection["box"],
                detection["distance"],
                detection["angle"],
                detection["confidence"],
                obj_type
            )
            new_obj.is_known_object = detection["is_known"]
            tracked_objects.append(new_obj)
    
    return tracked_objects

# Main function
def main():
    global MIN_DEPTH, MAX_DEPTH, EDGE_THRESHOLD
    
    print(f"Initializing camera (depth range: {MIN_DEPTH}m - {MAX_DEPTH}m)...")
    pipeline = rs.pipeline()
    config = rs.config()
    
    try:
        # Configure streams
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # Start streaming
        print("Starting camera pipeline...")
        pipeline_profile = pipeline.start(config)
        device = pipeline_profile.get_device()
        
        # Get depth scale for reference
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth Scale is: {depth_scale}")
        
        # Create align object
        align = rs.align(rs.stream.color)
        
        # Load YOLO model
        net, output_layers, classes = load_yolo_model()
        
        # Performance tracking
        frame_count = 0
        start_time = time.time()
        fps = 0
        
        # List to keep track of all detected objects
        tracked_objects = []
        
        # Detection mode (0: YOLO only, 1: Both YOLO & Obstruction, 2: Obstruction only)
        detection_mode = 1
        
        print("Starting detection loop...")
        while True:
            loop_start = time.time()
            
            try:
                # Wait for frames with timeout
                frames = pipeline.wait_for_frames(timeout_ms=5000)
            except Exception as e:
                print(f"Frame timeout: {e}")
                continue
            
            # Align frames
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create a copy of the color image for display
            display_image = color_image.copy()
            
            current_time = time.time()
            all_detections = []
            
            # Run object detection based on mode
            if detection_mode == 0 or detection_mode == 1:
                # Run YOLO detection
                yolo_detections = detect_objects_yolo(color_image, depth_frame, net, output_layers, classes)
                all_detections.extend(yolo_detections)
            
            # Run obstruction detection
            if detection_mode == 1 or detection_mode == 2:
                obstructions, depth_colormap, edge_display = detect_obstructions(depth_frame, color_image)
                all_detections.extend(obstructions)
            else:
                # Create empty depth and edge maps if not using obstruction detection
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03), 
                    cv2.COLORMAP_JET
                )
                edge_display = np.zeros_like(color_image)
            
            # Update tracked objects
            tracked_objects = update_tracked_objects(tracked_objects, all_detections, current_time)
            
            # Draw tracked objects on display image
            for obj in tracked_objects:
                if obj.is_stale(current_time):
                    continue  # Skip stale objects
                    
                x, y, w, h = obj.box
                color = obj.get_display_color()
                
                # Draw bounding box
                cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
                
                # Prepare label text
                if obj.is_known_object:
                    text = f"{obj.label} {obj.confidence:.2f} {obj.distance:.2f}m"
                else:
                    if obj.type == ObstructionType.VERTICAL:
                        text = f"Vertical {obj.distance:.2f}m {obj.angle:.1f}°"
                    elif obj.type == ObstructionType.HORIZONTAL:
                        text = f"Wall {obj.distance:.2f}m {obj.angle:.1f}°"
                    else:
                        text = f"Object {obj.distance:.2f}m {obj.angle:.1f}°"
                
                # Add background for better text visibility
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(display_image, (x, y - text_size[1] - 10), 
                             (x + text_size[0], y), (0, 0, 0), -1)
                
                # Draw text
                cv2.putText(display_image, text, (x, y - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # For vertical obstructions, draw a line indicating the angle
                if not obj.is_known_object and obj.type == ObstructionType.VERTICAL:
                    center_x = x + w // 2
                    center_y = y + h // 2
                    line_length = 100
                    end_x = int(center_x - line_length * math.sin(math.radians(obj.angle)))
                    end_y = int(center_y - line_length * math.cos(math.radians(obj.angle)))
                    cv2.line(display_image, (center_x, center_y), (end_x, end_y), (0, 255, 255), 2)
            
            # Create top-down view
            top_down_size = 240
            top_down = create_top_down_view(top_down_size, top_down_size, tracked_objects, MAX_DEPTH)
            
            # Position in bottom-right corner
            h, w = display_image.shape[:2]
            display_image[h-top_down_size:h, w-top_down_size:w] = top_down
            
            # Calculate FPS
            frame_count += 1
            if current_time - start_time >= 1.0:
                fps = frame_count / (current_time - start_time)
                frame_count = 0
                start_time = current_time
            
            # Create dashboard
            dashboard_height = 80
            dashboard = np.zeros((dashboard_height, w, 3), dtype=np.uint8)
            
            # Display mode, FPS, and object counts
            mode_text = ["YOLO Only", "YOLO + Depth", "Depth Only"][detection_mode]
            cv2.putText(dashboard, f"Mode: {mode_text} | FPS: {fps:.1f}", 
                      (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display object counts by category
            object_counts = {}
            for obj in tracked_objects:
                if not obj.is_stale(current_time):
                    key = obj.label if obj.is_known_object else obj.type.name
                    if key in object_counts:
                        object_counts[key] += 1
                    else:
                        object_counts[key] = 1
            
            # Format object counts for display
            count_text = " | ".join([f"{k}:{v}" for k, v in object_counts.items()])
            if count_text:
                cv2.putText(dashboard, count_text, 
                          (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(dashboard, "No objects detected", 
                          (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Combine dashboard with display image
            combined_image = np.vstack([dashboard, display_image])
            
            # Create visualization of processing steps
            if detection_mode == 1 or detection_mode == 2:
                # Scale down depth and edge images
                scale_factor = 0.5
                small_depth = cv2.resize(depth_colormap, (int(w*scale_factor), int(h*scale_factor)))
                small_edge = cv2.resize(edge_display, (int(w*scale_factor), int(h*scale_factor)))
                
                # Create header for processing view
                process_header = np.zeros((dashboard_height//2, int(w*scale_factor*2), 3), dtype=np.uint8)
                cv2.putText(process_header, "Depth Map | Edge Detection", 
                          (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Combine depth and edge images side by side
                processing_view = np.hstack([small_depth, small_edge])
                processing_view = np.vstack([process_header, processing_view])
                
                # Show processing steps in separate window
                cv2.namedWindow('Processing Steps', cv2.WINDOW_NORMAL)
                cv2.imshow('Processing Steps', processing_view)
            
            # Show final display
            cv2.namedWindow('Unified Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Unified Detection', combined_image)
            
            # Handle keyboard input
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break
            elif key == ord('m') or key == ord('M'):
                # Cycle through detection modes
                detection_mode = (detection_mode + 1) % 3
                print(f"Switched to mode: {['YOLO Only', 'YOLO + Depth', 'Depth Only'][detection_mode]}")
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
            elif key == ord('e'):
                EDGE_THRESHOLD = min(EDGE_THRESHOLD + 5, 255)
                print(f"Increased edge threshold to {EDGE_THRESHOLD}")
            elif key == ord('E'):
                EDGE_THRESHOLD = max(EDGE_THRESHOLD - 5, 10)
                print(f"Decreased edge threshold to {EDGE_THRESHOLD}")
            elif key == ord('c'):
                # Clear all tracked objects
                tracked_objects = []
                print("Cleared all tracked objects")
                
            # Calculate processing time and adjust sleep if needed
            loop_time = time.time() - loop_start
            if loop_time < 0.03:  # Target ~30fps max
                time.sleep(0.03 - loop_time)
    
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Application closed")

if __name__ == "__main__":
    main()