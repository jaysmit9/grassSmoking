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
MIN_DEPTH = 0.3  # Lower this to detect closer objects
MAX_DEPTH = 5.0  # Keep max depth at 5 meters

# Parameters for obstruction detection
EDGE_THRESHOLD = 30  # Lower for more sensitive edge detection
DEPTH_THRESHOLD = 0.05  # More sensitive to depth changes (in meters)
MIN_CONTOUR_AREA = 500  # Smaller minimum area to detect smaller objects
VERTICAL_RATIO_THRESHOLD = 1.5  # Height/width ratio for vertical classification
HORIZONTAL_RATIO_THRESHOLD = 0.67  # Height/width ratio for horizontal classification

# Add these new parameters
CLUSTER_DISTANCE = 0.10  # Maximum distance between depth points to be considered part of same object
BUCKET_DETECTION_SIZE = (0.3, 0.5)  # Expected size range for a bucket (width, height) in meters
WALL_MIN_WIDTH = 0.5  # Minimum width to consider something a wall (in meters)

# Function to calculate angle from center
def calculate_angle(x, width, hfov=66):  # Updated to 66-degree horizontal FOV for RealSense
    # Calculate normalized position from center (-1 to 1)
    normalized_pos = (x - width/2) / (width/2)
    # Convert to angle based on horizontal field of view
    angle = normalized_pos * (hfov/2)
    return angle

# Function to detect obstructions from depth data
def detect_obstructions_from_depth(depth_frame, color_image):
    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    
    # Create a normalized depth image for display
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    
    # Create a binary image by thresholding depths
    valid_depth = (depth_image > MIN_DEPTH * 1000) & (depth_image < MAX_DEPTH * 1000)
    depth_display = np.zeros_like(depth_colormap)
    depth_display[valid_depth] = depth_colormap[valid_depth]
    
    # Get dimensions
    height, width = depth_image.shape
    
    # Convert to meters for processing
    depth_float = depth_image.astype(float) / 1000.0
    
    # Handle NaN or infinite values if any
    depth_float = np.nan_to_num(depth_float)
    
    # Apply blur to reduce noise - Fix for OpenCV error
    # Convert to CV_8U for median blur (supported format)
    depth_uint8 = cv2.normalize(depth_float, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    depth_uint8 = cv2.medianBlur(depth_uint8, 5)
    
    # Convert back to float for gradient calculation
    depth_blurred = cv2.normalize(depth_uint8, None, 0, 1, cv2.NORM_MINMAX).astype(np.float32)
    depth_blurred = depth_blurred * np.max(depth_float)
    
    # Calculate gradients in both directions
    gx = cv2.Sobel(depth_blurred, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(depth_blurred, cv2.CV_32F, 0, 1, ksize=3)
    
    # Calculate gradient magnitude
    grad_mag = cv2.magnitude(gx, gy)
    
    # Normalize and threshold the gradient magnitude
    grad_mag = cv2.normalize(grad_mag, None, 0, 255, cv2.NORM_MINMAX)
    grad_mag = grad_mag.astype(np.uint8)
    
    # Apply threshold to get edges
    _, edge_binary = cv2.threshold(grad_mag, EDGE_THRESHOLD, 255, cv2.THRESH_BINARY)
    
    # Morphological operations to clean up the edges
    kernel = np.ones((3, 3), np.uint8)
    edge_binary = cv2.morphologyEx(edge_binary, cv2.MORPH_CLOSE, kernel)
    edge_binary = cv2.morphologyEx(edge_binary, cv2.MORPH_OPEN, kernel)
    
    # Find contours in the edge binary image
    contours, _ = cv2.findContours(edge_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Initialize list of obstructions
    obstructions = []
    
    # Process each contour
    for contour in contours:
        area = cv2.contourArea(contour)
        
        # Skip small contours
        if area < MIN_CONTOUR_AREA:
            continue
        
        # Get bounding box
        x, y, w, h = cv2.boundingRect(contour)
        
        # Skip if the box is too large (likely the whole scene)
        if w > width * 0.8 or h > height * 0.8:
            continue
            
        # Calculate center point
        center_x = x + w // 2
        center_y = y + h // 2
        
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
        
        # Skip if distance is outside our range
        if not (MIN_DEPTH <= dist <= MAX_DEPTH):
            continue
        
        # Calculate aspect ratio to classify the obstruction
        aspect_ratio = h / w if w > 0 else 999
        
        # Determine obstruction type based on aspect ratio
        if aspect_ratio > VERTICAL_RATIO_THRESHOLD:
            obstruction_type = ObstructionType.VERTICAL
            color = (0, 0, 255)  # Red for vertical obstructions
        elif aspect_ratio < HORIZONTAL_RATIO_THRESHOLD:
            obstruction_type = ObstructionType.HORIZONTAL
            color = (255, 0, 0)  # Blue for horizontal barriers
        else:
            obstruction_type = ObstructionType.UNKNOWN
            color = (255, 0, 255)  # Purple for unknown obstructions
        
        # Calculate angle from center
        angle = calculate_angle(center_x, width)
        
        # Draw box on color image
        cv2.rectangle(color_image, (x, y), (x + w, y + h), color, 2)
        
        # Annotate with details
        if obstruction_type == ObstructionType.VERTICAL:
            text = f"Vertical {dist:.2f}m {angle:.1f}°"
        elif obstruction_type == ObstructionType.HORIZONTAL:
            text = f"Horizontal {dist:.2f}m {angle:.1f}°"
        else:
            text = f"Unknown {dist:.2f}m {angle:.1f}°"
        
        cv2.putText(color_image, text, (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # For vertical obstructions, draw a line indicating the angle
        if obstruction_type == ObstructionType.VERTICAL:
            line_length = 100
            end_x = int(center_x - line_length * math.sin(math.radians(angle)))
            end_y = int(center_y - line_length * math.cos(math.radians(angle)))
            cv2.line(color_image, (center_x, center_y), (end_x, end_y), (0, 255, 255), 2)
        
        # Store this obstruction
        obstructions.append({
            "type": obstruction_type,
            "distance": dist,
            "angle": angle,
            "box": (x, y, w, h)
        })
    
    # Draw edges on display image
    edge_display = cv2.cvtColor(edge_binary, cv2.COLOR_GRAY2BGR)
    
    return color_image, depth_display, edge_display, obstructions

def detect_specific_obstructions(depth_frame, color_image):
    """Specialized function to detect specific objects like buckets and walls"""
    # Convert depth frame to numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    height, width = depth_image.shape
    
    # Create a depth map in meters
    depth_meters = depth_image.astype(float) / 1000.0
    
    # Create visualization of depth
    depth_colormap = cv2.applyColorMap(
        cv2.convertScaleAbs(depth_image, alpha=0.03), 
        cv2.COLORMAP_JET
    )
    
    # Create a copy of color image for display
    display_image = color_image.copy()
    
    # Create point cloud representation for analysis
    points = []
    for y in range(0, height, 4):  # Sample every 4th point for speed
        for x in range(0, width, 4):
            dist = depth_frame.get_distance(x, y)
            if MIN_DEPTH <= dist <= MAX_DEPTH:
                points.append((x, y, dist))
    
    # Skip if no valid points found
    if not points:
        return display_image, depth_colormap, np.zeros_like(color_image), []
    
    # Identify potential bucket areas (vertical cylindrical objects)
    bucket_candidates = []
    wall_candidates = []
    
    # Scan horizontally through depth image looking for vertical structures
    for x_start in range(0, width, width // 10):
        for x_end in range(x_start + width // 20, min(x_start + width // 5, width), width // 20):
            # Get vertical depth profile at this X coordinate
            vertical_profile = []
            for y in range(0, height, 4):
                dist1 = depth_frame.get_distance(x_start, y)
                dist2 = depth_frame.get_distance(x_end, y)
                if MIN_DEPTH <= dist1 <= MAX_DEPTH and MIN_DEPTH <= dist2 <= MAX_DEPTH:
                    # If depths are similar, might be same object
                    if abs(dist1 - dist2) < DEPTH_THRESHOLD:
                        avg_dist = (dist1 + dist2) / 2
                        vertical_profile.append((y, avg_dist))
            
            # Look for consistent vertical segments
            if len(vertical_profile) > 10:  # Need enough vertical points
                segments = []
                current_segment = [vertical_profile[0]]
                
                for i in range(1, len(vertical_profile)):
                    prev_y, prev_dist = current_segment[-1]
                    curr_y, curr_dist = vertical_profile[i]
                    
                    # If depth is similar and points are connected
                    if abs(curr_dist - prev_dist) < DEPTH_THRESHOLD and curr_y - prev_y < 12:
                        current_segment.append(vertical_profile[i])
                    else:
                        if len(current_segment) > 5:  # Segment needs minimum height
                            segments.append(current_segment)
                        current_segment = [vertical_profile[i]]
                
                if len(current_segment) > 5:
                    segments.append(current_segment)
                
                # Process found segments
                for segment in segments:
                    y_values = [y for y, _ in segment]
                    depth_values = [d for _, d in segment]
                    
                    min_y = min(y_values)
                    max_y = max(y_values)
                    avg_depth = sum(depth_values) / len(depth_values)
                    
                    # Estimate physical height based on depth
                    segment_height = (max_y - min_y) * avg_depth / 400  # Approximate conversion
                    segment_width = (x_end - x_start) * avg_depth / 400  # Approximate conversion
                    
                    # Check if dimensions match expected bucket size
                    if (BUCKET_DETECTION_SIZE[0] * 0.7 <= segment_width <= BUCKET_DETECTION_SIZE[0] * 1.3 and
                        BUCKET_DETECTION_SIZE[1] * 0.7 <= segment_height <= BUCKET_DETECTION_SIZE[1] * 1.3):
                        bucket_candidates.append({
                            "x1": x_start,
                            "x2": x_end,
                            "y1": min_y,
                            "y2": max_y,
                            "depth": avg_depth,
                            "type": ObstructionType.VERTICAL,
                            "confidence": 0.8
                        })
                    
                    # Check if it could be a wall (wide and consistent depth)
                    if segment_width > WALL_MIN_WIDTH:
                        # Look for consistent depth across width
                        consistent_depth = True
                        reference_depth = depth_frame.get_distance(x_start, (min_y + max_y) // 2)
                        
                        for check_x in range(x_start, x_end, 10):
                            check_depth = depth_frame.get_distance(check_x, (min_y + max_y) // 2)
                            if abs(check_depth - reference_depth) > DEPTH_THRESHOLD:
                                consistent_depth = False
                                break
                        
                        if consistent_depth:
                            wall_candidates.append({
                                "x1": x_start,
                                "x2": x_end,
                                "y1": min_y,
                                "y2": max_y,
                                "depth": avg_depth,
                                "type": ObstructionType.HORIZONTAL,
                                "confidence": 0.8
                            })
    
    # Now scan vertically for horizontal structures (mainly walls)
    for y_start in range(0, height, height // 10):
        for y_end in range(y_start + height // 20, min(y_start + height // 5, height), height // 20):
            # Get horizontal depth profile
            horizontal_profile = []
            for x in range(0, width, 4):
                dist1 = depth_frame.get_distance(x, y_start)
                dist2 = depth_frame.get_distance(x, y_end)
                if MIN_DEPTH <= dist1 <= MAX_DEPTH and MIN_DEPTH <= dist2 <= MAX_DEPTH:
                    if abs(dist1 - dist2) < DEPTH_THRESHOLD:
                        avg_dist = (dist1 + dist2) / 2
                        horizontal_profile.append((x, avg_dist))
            
            # Look for consistent horizontal segments
            if len(horizontal_profile) > 15:  # Need enough horizontal points
                segments = []
                current_segment = [horizontal_profile[0]]
                
                for i in range(1, len(horizontal_profile)):
                    prev_x, prev_dist = current_segment[-1]
                    curr_x, curr_dist = horizontal_profile[i]
                    
                    # If depth is similar and points are connected
                    if abs(curr_dist - prev_dist) < DEPTH_THRESHOLD and curr_x - prev_x < 12:
                        current_segment.append(horizontal_profile[i])
                    else:
                        if len(current_segment) > 10:  # Segment needs minimum width
                            segments.append(current_segment)
                        current_segment = [horizontal_profile[i]]
                
                if len(current_segment) > 10:
                    segments.append(current_segment)
                
                # Process found segments
                for segment in segments:
                    x_values = [x for x, _ in segment]
                    depth_values = [d for _, d in segment]
                    
                    min_x = min(x_values)
                    max_x = max(x_values)
                    avg_depth = sum(depth_values) / len(depth_values)
                    
                    # Estimate physical width based on depth
                    segment_width = (max_x - min_x) * avg_depth / 400  # Approximate conversion
                    
                    # Check if it could be a wall (wide and consistent depth)
                    if segment_width > WALL_MIN_WIDTH:
                        wall_candidates.append({
                            "x1": min_x,
                            "x2": max_x,
                            "y1": y_start,
                            "y2": y_end,
                            "depth": avg_depth,
                            "type": ObstructionType.HORIZONTAL,
                            "confidence": 0.9
                        })
    
    # Combine and filter candidates to avoid duplicates
    all_candidates = bucket_candidates + wall_candidates
    filtered_candidates = []
    
    # Non-maximum suppression to remove duplicates
    while all_candidates:
        best = max(all_candidates, key=lambda x: x["confidence"])
        all_candidates.remove(best)
        
        # Add to filtered list
        filtered_candidates.append(best)
        
        # Remove any overlapping candidates
        all_candidates = [c for c in all_candidates if not (
            abs(c["depth"] - best["depth"]) < 0.3 and  # Similar depth
            abs((c["x1"] + c["x2"])/2 - (best["x1"] + best["x2"])/2) < width/4 and  # Similar X center
            abs((c["y1"] + c["y2"])/2 - (best["y1"] + best["y2"])/2) < height/4  # Similar Y center
        )]
    
    # Draw candidates and prepare results
    obstructions = []
    edge_display = np.zeros_like(color_image)
    
    for candidate in filtered_candidates:
        x1, x2, y1, y2 = candidate["x1"], candidate["x2"], candidate["y1"], candidate["y2"]
        dist = candidate["depth"]
        obs_type = candidate["type"]
        
        # Calculate center for angle calculation
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # Calculate angle from center
        angle = calculate_angle(center_x, width)
        
        # Choose color based on type
        if obs_type == ObstructionType.VERTICAL:
            color = (0, 0, 255)  # Red for vertical (buckets)
            label = "Bucket"
        elif obs_type == ObstructionType.HORIZONTAL:
            color = (255, 0, 0)  # Blue for horizontal (walls)
            label = "Wall"
        else:
            color = (255, 0, 255)  # Purple for unknown
            label = "Object"
        
        # Draw on display image
        cv2.rectangle(display_image, (x1, y1), (x2, y2), color, 2)
        
        # Draw also on edge display for visualization
        cv2.rectangle(edge_display, (x1, y1), (x2, y2), color, 2)
        
        # Create text with details
        text = f"{label} {dist:.2f}m {angle:.1f}°"
        cv2.putText(display_image, text, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # Add to obstructions list
        obstructions.append({
            "type": obs_type,
            "distance": dist,
            "angle": angle,
            "box": (x1, y1, x2-x1, y2-y1)
        })
    
    return display_image, depth_colormap, edge_display, obstructions

# Function to create a top-down view of obstructions
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
        distance = obs["distance"]
        angle = obs["angle"]
        obs_type = obs["type"]
        
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
            
            # Add distance label
            cv2.putText(top_view, f"{distance:.1f}m", (x+5, y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return top_view

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
        
        # Performance tracking
        frame_count = 0
        start_time = time.time()
        fps = 0
        
        # Detection mode (0: standard, 1: specific)
        detection_mode = 1
        
        print("Starting obstruction detection loop...")
        while True:
            # Wait for frames
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            
            # Align frames
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            
            # Detect obstructions based on selected mode
            if detection_mode == 0:
                detection_result, depth_display, edge_display, obstructions = detect_obstructions_from_depth(
                    depth_frame, color_image.copy())
            else:
                detection_result, depth_display, edge_display, obstructions = detect_specific_obstructions(
                    depth_frame, color_image.copy())
            
            # Create top-down view
            if obstructions:
                top_down_size = 240
                top_down = create_top_down_view(top_down_size, top_down_size, obstructions, MAX_DEPTH)
                
                # Position in bottom-right corner of detection result
                h, w = detection_result.shape[:2]
                detection_result[h-top_down_size:h, w-top_down_size:w] = top_down
            
            # Calculate FPS
            frame_count += 1
            current_time = time.time()
            if current_time - start_time >= 1.0:
                fps = frame_count / (current_time - start_time)
                frame_count = 0
                start_time = current_time
            
            # Create dashboard at top
            dashboard_height = 40
            dashboard = np.zeros((dashboard_height, color_image.shape[1], 3), dtype=np.uint8)
            
            # Display FPS, mode and obstruction count
            mode_text = "Specific" if detection_mode == 1 else "General"
            cv2.putText(dashboard, f"FPS: {fps:.1f} | Mode: {mode_text} | Obstructions: {len(obstructions)}", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Combine images
            detection_with_dashboard = np.vstack([dashboard, detection_result])
            
            # Scale down depth and edge images to fit side by side
            scale_factor = 0.5
            h, w = detection_result.shape[:2]
            small_depth = cv2.resize(depth_display, (int(w*scale_factor), int(h*scale_factor)))
            small_edge = cv2.resize(edge_display, (int(w*scale_factor), int(h*scale_factor)))
            
            # Combine depth and edge images side by side
            processing_view = np.hstack([small_depth, small_edge])
            processing_h, processing_w = processing_view.shape[:2]
            
            # Add a header to processing view
            process_header = np.zeros((dashboard_height, processing_w, 3), dtype=np.uint8)
            cv2.putText(process_header, "Depth Map | Detection Debug", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            processing_view = np.vstack([process_header, processing_view])
            
            # Show images
            cv2.namedWindow('Obstruction Detection', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstruction Detection', detection_with_dashboard)
            
            cv2.namedWindow('Processing Steps', cv2.WINDOW_NORMAL)
            cv2.imshow('Processing Steps', processing_view)
            
            # Handle keyboard input
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                break
            elif key == ord('m'):  # Toggle detection mode
                detection_mode = 1 - detection_mode
                print(f"Switched to {'specific' if detection_mode == 1 else 'general'} detection mode")
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
            
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Application closed")

if __name__ == "__main__":
    main()