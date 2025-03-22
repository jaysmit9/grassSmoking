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
    VERTICAL = 1    # Like trees, poles
    HORIZONTAL = 2  # Like walls, fences, hedges
    UNKNOWN = 3     # Unclassified obstruction

# Define min and max depths in meters
MIN_DEPTH = 0.3  # Objects closer than 0.3 meters will be ignored
MAX_DEPTH = 5.0  # Objects farther than 5.0 meters will be ignored

# Set detection expiration time (in seconds)
DETECTION_TIMEOUT = 1.0  # Detections older than this will be removed

# Add parameters for obstruction detection
OBSTRUCTION_CONFIDENCE = 0.65  # Higher threshold for obstruction detection
VERTICAL_RATIO_THRESHOLD = 1.8  # Height/width ratio for vertical classification
HORIZONTAL_RATIO_THRESHOLD = 0.6  # Height/width ratio for horizontal classification

# Function to calculate angle from center
def calculate_angle(x, width, hfov=69):  # 69 is typical horizontal FOV for RealSense
    # Calculate normalized position from center (-1 to 1)
    normalized_pos = (x - width/2) / (width/2)
    # Convert to angle based on horizontal field of view
    angle = normalized_pos * (hfov/2)
    return angle

# Function to detect and classify obstructions
def detect_obstruction(box, depth_frame, width, height):
    x, y, w, h = box
    
    # Calculate aspect ratio
    aspect_ratio = h / w if w > 0 else 999
    
    # Determine obstruction type based on aspect ratio
    if aspect_ratio > VERTICAL_RATIO_THRESHOLD:
        obstruction_type = ObstructionType.VERTICAL
    elif aspect_ratio < HORIZONTAL_RATIO_THRESHOLD:
        obstruction_type = ObstructionType.HORIZONTAL
    else:
        obstruction_type = ObstructionType.UNKNOWN
    
    # Calculate center point
    center_x = x + w//2
    center_y = y + h//2
    
    # Get distance at center point
    dist = depth_frame.get_distance(center_x, center_y)
    
    # If center point has no valid depth, sample multiple points
    if dist == 0:
        depths = []
        # Sample multiple points in the box
        for i in range(5):
            for j in range(5):
                sample_x = x + (w * i) // 5
                sample_y = y + (h * j) // 5
                sample_dist = depth_frame.get_distance(sample_x, sample_y)
                if sample_dist > 0:
                    depths.append(sample_dist)
        
        if depths:
            dist = np.median(depths)
    
    # Calculate angle from center (negative = left, positive = right)
    angle = calculate_angle(center_x, width)
    
    return obstruction_type, dist, angle

# Check if the required files exist
required_files = ["yolov3-tiny.weights", "yolov3-tiny.cfg", "coco.names"]
for file in required_files:
    if not os.path.isfile(file):
        print(f"Error: {file} not found in the current directory")
        sys.exit(1)

print(f"Initializing camera (depth range: {MIN_DEPTH}m - {MAX_DEPTH}m)...")
# Configure streams
pipeline = rs.pipeline()
config = rs.config()

try:
    # Configure streams with more relaxed timing requirements
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    
    # Start streaming
    print("Starting camera pipeline...")
    pipeline_profile = pipeline.start(config)
    device = pipeline_profile.get_device()
    
    # Reset the device before starting to clear any previous state
    try:
        print("Resetting camera to ensure clean startup...")
        device.hardware_reset()
        time.sleep(3)  # Give device time to reset
        print("Camera reset complete, starting streams")
    except:
        print("Could not reset device, continuing anyway")
    
    print("Camera pipeline started successfully")
    
    # Get depth scale for reference
    depth_sensor = pipeline_profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale is: {depth_scale}")
    
    # Create align object
    align = rs.align(rs.stream.color)
    
    # Load YOLO
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
    
    # Performance tracking
    frame_count = 0
    start_time = time.time()
    fps = 0
    
    # Variables to store detection results between frames
    last_boxes = []
    last_confidences = []
    last_class_ids = []
    last_distances = []
    last_detection_time = []
    
    print("Starting detection loop...")
    consecutive_timeouts = 0
    max_consecutive_timeouts = 5
    
    try:
        while True:
            loop_start = time.time()
            current_time = time.time()
            
            try:
                # Wait for frames with increased timeout
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                consecutive_timeouts = 0  # Reset timeout counter on success
            except Exception as e:
                consecutive_timeouts += 1
                print(f"Frame timeout ({consecutive_timeouts}/{max_consecutive_timeouts}): {e}")
                
                # If we've had too many consecutive timeouts, try to recover
                if consecutive_timeouts >= max_consecutive_timeouts:
                    print("Too many consecutive timeouts, attempting to recover...")
                    try:
                        pipeline.stop()
                        time.sleep(1)
                        pipeline = rs.pipeline()
                        pipeline.start(config)
                        consecutive_timeouts = 0
                        print("Pipeline restarted successfully")
                    except Exception as restart_error:
                        print(f"Error restarting pipeline: {restart_error}")
                continue
            
            # Try to align frames safely
            try:
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
            except Exception as e:
                print(f"Frame alignment error: {e}")
                # Fall back to unaligned frames
                try:
                    depth_frame = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                except:
                    print("Could not extract frames")
                    continue
            
            if not depth_frame or not color_frame:
                print("Missing frames, continuing...")
                continue
            
            # Convert images to numpy arrays - only color for display
            try:
                color_image = np.asanyarray(color_frame.get_data())
                height, width, channels = color_image.shape
            except Exception as e:
                print(f"Error converting frames to numpy arrays: {e}")
                continue
            
            # Process every other frame for better performance
            if frame_count % 2 == 0:  # Only process every other frame
                # YOLO object detection
                try:
                    # Use smaller input size for better performance
                    blob = cv2.dnn.blobFromImage(color_image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
                    net.setInput(blob)
                    outs = net.forward(output_layers)
                    
                    # Process detections
                    class_ids = []
                    confidences = []
                    boxes = []
                    distances = []
                    detection_times = []
                    
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
                                
                                # Get distance before deciding to keep the object
                                try:
                                    dist = depth_frame.get_distance(center_x, center_y)
                                    
                                    # Simplified sampling - just check a few points
                                    if dist == 0:
                                        # Fall back to sampling only if center point is invalid
                                        dist_samples = []
                                        for dx, dy in [(-5, -5), (5, 5), (-5, 5), (5, -5)]:
                                            px = min(max(0, center_x + dx), width-1)
                                            py = min(max(0, center_y + dy), height-1)
                                            d = depth_frame.get_distance(px, py)
                                            if d > 0:
                                                dist_samples.append(d)
                                        
                                        if dist_samples:
                                            dist = np.median(dist_samples)
                                    
                                    # Filter objects based on distance
                                    if dist > 0 and MIN_DEPTH <= dist <= MAX_DEPTH:
                                        boxes.append([x, y, w, h])
                                        confidences.append(float(confidence))
                                        class_ids.append(class_id)
                                        distances.append(dist)
                                        detection_times.append(current_time)
                                        
                                except Exception as e:
                                    # Still include the object if we can't get distance
                                    boxes.append([x, y, w, h])
                                    confidences.append(float(confidence))
                                    class_ids.append(class_id)
                                    distances.append(0)
                                    detection_times.append(current_time)
                    
                    # Apply non-max suppression
                    if boxes:
                        try:
                            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.6, 0.4)
                            indexes = indexes.flatten() if isinstance(indexes, np.ndarray) else indexes
                            
                            # Update stored detections
                            last_boxes = [boxes[i] for i in indexes]
                            last_confidences = [confidences[i] for i in indexes]
                            last_class_ids = [class_ids[i] for i in indexes]
                            last_distances = [distances[i] for i in indexes]
                            last_detection_time = [detection_times[i] for i in indexes]
                        except Exception as e:
                            print(f"Error in NMS: {e}")
                    else:
                        # No detections in this frame - clear lists if timeout has elapsed
                        if not last_boxes or (current_time - max(last_detection_time) > DETECTION_TIMEOUT if last_detection_time else True):
                            last_boxes = []
                            last_confidences = []
                            last_class_ids = []
                            last_distances = []
                            last_detection_time = []
                except Exception as e:
                    print(f"Error in object detection: {e}")
            else:
                # For frames where we don't run detection, check for stale detections
                if last_detection_time:
                    # Get indices of non-stale detections (within timeout)
                    valid_indices = [i for i, t in enumerate(last_detection_time) 
                                     if current_time - t <= DETECTION_TIMEOUT]
                    
                    # Filter out stale detections
                    if valid_indices:
                        last_boxes = [last_boxes[i] for i in valid_indices]
                        last_confidences = [last_confidences[i] for i in valid_indices]
                        last_class_ids = [last_class_ids[i] for i in valid_indices]
                        last_distances = [last_distances[i] for i in valid_indices]
                        last_detection_time = [last_detection_time[i] for i in valid_indices]
                    else:
                        # All detections are stale, clear everything
                        last_boxes = []
                        last_confidences = []
                        last_class_ids = []
                        last_distances = []
                        last_detection_time = []
            
            # REPLACE THE DISPLAY CODE SECTION WITH THIS CODE
            # ===============================================

            # Create a copy of the color image for the main display
            display_image = color_image.copy()

            # Prepare object summary metrics
            detection_summary = []
            class_counts = {}

            # Process detection results for display
            for i in range(len(last_boxes)):
                x, y, w, h = last_boxes[i]
                label = str(classes[last_class_ids[i]])
                confidence = last_confidences[i]
                dist = last_distances[i]
                
                # Skip if distance is outside our range
                if not (MIN_DEPTH <= dist <= MAX_DEPTH) and dist != 0:
                    continue
                
                # Check if this is an obstruction (wall, fence, or unknown object with high confidence)
                is_obstruction = False
                obstruction_type = ObstructionType.UNKNOWN
                obstruction_angle = 0
                
                # List of classes that could be obstructions
                obstruction_classes = ['wall', 'fence', 'tree', 'pole']
                
                if label in obstruction_classes and confidence > OBSTRUCTION_CONFIDENCE:
                    is_obstruction = True
                    obstruction_type, obstruction_dist, obstruction_angle = detect_obstruction(
                        [x, y, w, h], depth_frame, width, height)
                    
                    # Add to a separate list of obstructions
                    if 'obstructions' not in class_counts:
                        class_counts['obstructions'] = 1
                    else:
                        class_counts['obstructions'] += 1
                
                # Update class count for summary
                if label in class_counts:
                    class_counts[label] += 1
                else:
                    class_counts[label] = 1
                
                # Add to summary list
                detection_summary.append((label, dist, confidence))
                
                # Choose color based on type of object
                if is_obstruction:
                    if obstruction_type == ObstructionType.VERTICAL:
                        color = (0, 0, 255)  # Red for vertical obstructions
                    elif obstruction_type == ObstructionType.HORIZONTAL:
                        color = (255, 0, 0)  # Blue for horizontal barriers
                    else:
                        color = (255, 0, 255)  # Purple for unknown obstructions
                else:
                    # Original color scheme for non-obstructions
                    if dist == 0:
                        color = (255, 255, 255)  # White for unknown distance
                    elif dist < 1.0:
                        color = (0, 0, 255)      # Red for close objects
                    elif dist < 2.0:
                        color = (0, 255, 255)    # Yellow for medium distance
                    else:
                        color = (0, 255, 0)      # Green for far objects
                
                # Draw bounding box with color based on distance
                cv2.rectangle(display_image, (x, y), (x + w, y + h), color, 2)
                
                # Add background for better text visibility
                if is_obstruction:
                    # Add obstruction type and angle information
                    text = f"{label} {dist:.2f}m"
                    if obstruction_type == ObstructionType.VERTICAL:
                        text += f" Vert {obstruction_angle:.1f}°"
                    elif obstruction_type == ObstructionType.HORIZONTAL:
                        text += f" Horiz {obstruction_angle:.1f}°"
                else:
                    text = f"{label} {confidence:.2f} {dist:.2f}m"
                
                text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(display_image, (x, y - text_size[1] - 10), 
                             (x + text_size[0], y), (0, 0, 0), -1)
                
                # Draw text
                cv2.putText(display_image, text, (x, y - 5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                # For vertical obstructions, draw a line indicating the angle
                if is_obstruction and obstruction_type == ObstructionType.VERTICAL:
                    # Draw a line showing the angle from center
                    center_x = x + w//2
                    center_y = y + h//2
                    line_length = 100
                    end_x = int(center_x - line_length * math.sin(math.radians(obstruction_angle)))
                    end_y = int(center_y - line_length * math.cos(math.radians(obstruction_angle)))
                    cv2.line(display_image, (center_x, center_y), (end_x, end_y), (0, 255, 255), 2)

            # Create black dashboard area at the top (with fixed height)
            dashboard_height = 100
            dashboard = np.zeros((dashboard_height, width, 3), dtype=np.uint8)

            # Add FPS counter to dashboard
            cv2.putText(dashboard, f"FPS: {fps:.1f}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Add range info to dashboard
            cv2.putText(dashboard, f"Range: {MIN_DEPTH:.1f}m - {MAX_DEPTH:.1f}m", (width - 250, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Format and display object counts
            obj_text = "Objects: "
            if class_counts:
                obj_text += ", ".join([f"{cls}({count})" for cls, count in sorted(class_counts.items())])
            else:
                obj_text += "None"
            cv2.putText(dashboard, obj_text[:width//10], (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Display closest object info
            if detection_summary:
                valid_distances = [(label, dist, conf) for label, dist, conf in detection_summary if dist > 0]
                if valid_distances:
                    closest = min(valid_distances, key=lambda x: x[1])
                    cv2.putText(dashboard, f"Closest: {closest[0]} at {closest[1]:.2f}m", 
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(dashboard, "No distance data available", 
                                (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(dashboard, "No objects detected", 
                            (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Add to your dashboard section

            # Display obstruction information
            if 'obstructions' in class_counts and class_counts['obstructions'] > 0:
                # Find all obstructions and their details
                obstruction_info = []
                for i in range(len(last_boxes)):
                    label = str(classes[last_class_ids[i]])
                    if label in ['wall', 'fence', 'tree', 'pole']:
                        x, y, w, h = last_boxes[i]
                        dist = last_distances[i]
                        obstruction_type, obstruction_dist, obstruction_angle = detect_obstruction(
                            [x, y, w, h], depth_frame, width, height)
                        obstruction_info.append((label, obstruction_type, obstruction_dist, obstruction_angle))
                
                # Sort obstructions by distance
                obstruction_info.sort(key=lambda x: x[2])
                
                # Display closest obstruction
                if obstruction_info:
                    closest = obstruction_info[0]
                    obstruction_text = f"Obstruction: {closest[0]} "
                    if closest[1] == ObstructionType.VERTICAL:
                        obstruction_text += f"(Vertical), {closest[2]:.2f}m, {closest[3]:.1f}°"
                    elif closest[1] == ObstructionType.HORIZONTAL:
                        obstruction_text += f"(Horizontal), {closest[2]:.2f}m, {closest[3]:.1f}°"
                    else:
                        obstruction_text += f"{closest[2]:.2f}m, {closest[3]:.1f}°"
                    
                    cv2.putText(dashboard, obstruction_text, (10, 120), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Combine dashboard and display image
            combined_image = np.vstack([dashboard, display_image])

            # Add a top-down view panel showing obstructions

            # Create a top-down view
            def create_top_down_view(width, height, obstructions, max_distance=5.0):
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
                
                # Draw obstructions
                for obs in obstructions:
                    label, obs_type, distance, angle = obs
                    if distance > 0 and distance <= max_distance:
                        # Convert to cartesian coordinates
                        rad = math.radians(-angle)  # Negative because screen coordinates
                        x = int(center_x + distance * (height - 40) / max_distance * math.sin(rad))
                        y = int(center_y - distance * (height - 40) / max_distance * math.cos(rad))
                        
                        # Draw different markers for vertical vs horizontal
                        if obs_type == ObstructionType.VERTICAL:
                            cv2.circle(top_view, (x, y), 7, (0, 0, 255), -1)  # Red circle
                        elif obs_type == ObstructionType.HORIZONTAL:
                            # Draw a small line perpendicular to the angle
                            perp_rad = rad + math.pi/2
                            line_length = 15
                            x1 = int(x + line_length * math.sin(perp_rad))
                            y1 = int(y - line_length * math.cos(perp_rad))
                            x2 = int(x - line_length * math.sin(perp_rad))
                            y2 = int(y - line_length * math.cos(perp_rad))
                            cv2.line(top_view, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue line
                        else:
                            cv2.rectangle(top_view, (x-5, y-5), (x+5, y+5), (255, 0, 255), -1)  # Purple square
                        
                        # Add label
                        cv2.putText(top_view, f"{label[:3]}", (x+5, y), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                return top_view

            # In the main loop, collect obstruction info and create top-down view
            obstruction_info = []
            for i in range(len(last_boxes)):
                label = str(classes[last_class_ids[i]])
                if label in ['wall', 'fence', 'tree', 'pole']:
                    x, y, w, h = last_boxes[i]
                    dist = last_distances[i]
                    obstruction_type, obstruction_dist, obstruction_angle = detect_obstruction(
                        [x, y, w, h], depth_frame, width, height)
                    obstruction_info.append((label, obstruction_type, obstruction_dist, obstruction_angle))

            # Create top-down view if we have obstructions
            if obstruction_info:
                top_down_size = 240  # Size of the top-down view
                top_down = create_top_down_view(top_down_size, top_down_size, obstruction_info, MAX_DEPTH)
                
                # Position in bottom-right corner
                display_image[height-top_down_size:height, width-top_down_size:width] = top_down

            # Show ONLY the combined image - nothing else
            cv2.namedWindow('Object Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Object Detection', combined_image)
            cv2.destroyWindow('RealSense Object Detection')  # Close any other windows

            # Calculate and update FPS (just the counter part)
            frame_count += 1
            if frame_count >= 30:
                current_time = time.time()
                fps = frame_count / (current_time - start_time)
                frame_count = 0
                start_time = current_time
            
            # Handle keyboard input
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break
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
                
    except KeyboardInterrupt:
        print("Keyboard interrupt, stopping...")
    
except Exception as e:
    print(f"Setup error: {e}")
    traceback.print_exc()
    
finally:
    # Stop streaming
    print("Stopping pipeline...")
    try:
        pipeline.stop()
    except Exception as e:
        print(f"Error stopping pipeline: {e}")
        
    cv2.destroyAllWindows()
    print("Application closed")