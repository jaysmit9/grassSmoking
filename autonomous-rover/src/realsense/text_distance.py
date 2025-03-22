import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
from enum import Enum
import curses
import math

# Define min and max depths in meters
MIN_DEPTH = 0.3
MAX_DEPTH = 5.0

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
    
    return pipeline, align

def get_frames(pipeline, align):
    """Get aligned frames from the camera"""
    frames = pipeline.wait_for_frames(timeout_ms=5000)
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not depth_frame or not color_frame:
        return None, None
    
    return depth_frame, color_frame

def analyze_depth_regions(depth_frame, width=640, height=480, regions=8):
    """Analyze depth in grid regions and report statistics"""
    depth_data = np.asanyarray(depth_frame.get_data())
    
    # Create a grid to analyze depth data in different regions
    region_width = width // regions
    region_height = height // regions
    
    results = []
    
    # Analyze each region
    for y_idx in range(regions):
        for x_idx in range(regions):
            x_start = x_idx * region_width
            y_start = y_idx * region_height
            x_end = x_start + region_width
            y_end = y_start + region_height
            
            # Extract region data
            region_data = depth_data[y_start:y_end, x_start:x_end]
            
            # Convert to meters and filter out zero/invalid values
            region_meters = region_data.astype(float) / 1000.0
            valid_depths = region_meters[(region_meters > 0) & 
                                        (region_meters >= MIN_DEPTH) & 
                                        (region_meters <= MAX_DEPTH)]
            
            # Calculate stats if we have valid data
            if len(valid_depths) > 0:
                avg_depth = np.mean(valid_depths)
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                valid_percentage = len(valid_depths) / (region_width * region_height) * 100
            else:
                avg_depth = 0
                min_depth = 0
                max_depth = 0
                valid_percentage = 0
            
            results.append({
                "position": (x_idx, y_idx),
                "avg_depth": avg_depth,
                "min_depth": min_depth,
                "max_depth": max_depth,
                "valid_percentage": valid_percentage
            })
    
    return results

def scan_for_objects(depth_frame, width=640, height=480):
    """Scan for potential objects by looking for depth discontinuities"""
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_meters = depth_image.astype(float) / 1000.0
    
    # Replace invalid values (0) with NaN for better processing
    depth_meters[depth_meters == 0] = np.nan
    
    # List to store detected objects
    objects = []
    
    # Horizontal scan (middle of the image)
    middle_y = height // 2
    horizontal_scan = depth_meters[middle_y, :]
    
    # Find transitions in horizontal scan
    h_transitions = []
    last_valid_depth = None
    
    for x in range(width):
        current_depth = horizontal_scan[x]
        
        # Skip invalid depths
        if np.isnan(current_depth) or current_depth < MIN_DEPTH or current_depth > MAX_DEPTH:
            continue
            
        # Record transitions
        if last_valid_depth is not None:
            depth_diff = abs(current_depth - last_valid_depth)
            if depth_diff > 0.15:  # Threshold for significant depth change
                h_transitions.append((x, last_valid_depth, current_depth))
                
        last_valid_depth = current_depth
    
    # Vertical scan (middle of the image)
    middle_x = width // 2
    vertical_scan = depth_meters[:, middle_x]
    
    # Find transitions in vertical scan
    v_transitions = []
    last_valid_depth = None
    
    for y in range(height):
        current_depth = vertical_scan[y]
        
        # Skip invalid depths
        if np.isnan(current_depth) or current_depth < MIN_DEPTH or current_depth > MAX_DEPTH:
            continue
            
        # Record transitions
        if last_valid_depth is not None:
            depth_diff = abs(current_depth - last_valid_depth)
            if depth_diff > 0.15:  # Threshold for significant depth change
                v_transitions.append((y, last_valid_depth, current_depth))
                
        last_valid_depth = current_depth
    
    # Additional scan patterns (diagonal and grid sampling)
    # Diagonal top-left to bottom-right
    diag1_transitions = scan_diagonal(depth_meters, 0, 0, width, height)
    
    # Diagonal top-right to bottom-left
    diag2_transitions = scan_diagonal(depth_meters, width-1, 0, -width, height)
    
    # Find depth clusters
    center_points = []
    
    # Sample grid points
    for y in range(0, height, 20):
        for x in range(0, width, 20):
            depth = depth_frame.get_distance(x, y)
            if MIN_DEPTH <= depth <= MAX_DEPTH:
                center_points.append((x, y, depth))
    
    # Group nearby points with similar depths
    clusters = []
    
    while center_points:
        point = center_points.pop(0)
        x, y, depth = point
        
        # Create new cluster with this point
        cluster = [point]
        
        # Find all points that belong to this cluster
        i = 0
        while i < len(center_points):
            cx, cy, cdepth = center_points[i]
            
            # Check if this point is close to our cluster (in space and depth)
            if (abs(cx - x) < 40 and abs(cy - y) < 40 and abs(cdepth - depth) < 0.2):
                cluster.append(center_points.pop(i))
            else:
                i += 1
        
        # Add cluster if it has enough points
        if len(cluster) >= 3:
            # Calculate average position and depth
            avg_x = sum(p[0] for p in cluster) / len(cluster)
            avg_y = sum(p[1] for p in cluster) / len(cluster)
            avg_depth = sum(p[2] for p in cluster) / len(cluster)
            
            # Calculate angle from center with correct FOV
            hfov = 66  # 66 degree horizontal field of view
            normalized_pos = (avg_x - width/2) / (width/2)
            angle = normalized_pos * (hfov/2)
            
            clusters.append({
                "center": (avg_x, avg_y),
                "depth": avg_depth,
                "points": len(cluster),
                "angle": angle
            })
    
    return {
        "horizontal_transitions": h_transitions,
        "vertical_transitions": v_transitions,
        "diagonal1_transitions": diag1_transitions,
        "diagonal2_transitions": diag2_transitions,
        "clusters": clusters
    }

def scan_diagonal(depth_meters, start_x, start_y, dx, dy):
    """Scan along a diagonal line and find depth transitions"""
    height, width = depth_meters.shape
    transitions = []
    last_valid_depth = None
    
    # Calculate number of steps
    steps = min(abs(dx), abs(dy))
    
    for i in range(steps):
        # Calculate position
        x = start_x + (dx * i) // steps
        y = start_y + (dy * i) // steps
        
        # Check bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            continue
            
        current_depth = depth_meters[y, x]
        
        # Skip invalid depths
        if np.isnan(current_depth) or current_depth < MIN_DEPTH or current_depth > MAX_DEPTH:
            continue
            
        # Record transitions
        if last_valid_depth is not None:
            depth_diff = abs(current_depth - last_valid_depth)
            if depth_diff > 0.15:  # Threshold for significant depth change
                transitions.append((i, last_valid_depth, current_depth))
                
        last_valid_depth = current_depth
    
    return transitions

def display_text_view(regions_data, scan_results):
    """Display depth data in text format"""
    os.system('cls' if os.name == 'nt' else 'clear')
    
    # Print header
    print("\n === DEPTH DATA ANALYSIS ===")
    print(f" Range: {MIN_DEPTH}m - {MAX_DEPTH}m")
    print(" Grid Analysis (8x8):")
    
    # Print grid headers
    print("\n    ", end="")
    for x in range(8):
        print(f"   {x}   ", end="")
    print("\n   +", end="")
    for x in range(8):
        print("-------", end="")
    print("+")
    
    # Print grid data
    for y in range(8):
        print(f" {y} |", end="")
        for x in range(8):
            region = next(r for r in regions_data if r["position"] == (x, y))
            
            if region["avg_depth"] > 0:
                print(f" {region['avg_depth']:.2f}m ", end="")
            else:
                print("   -   ", end="")
        print("|")
    
    print("   +", end="")
    for x in range(8):
        print("-------", end="")
    print("+")
    
    # Print detected transition points
    print("\n === DEPTH TRANSITIONS ===")
    print(" Horizontal (middle row):")
    for i, (x, d1, d2) in enumerate(scan_results["horizontal_transitions"]):
        print(f"  {i+1}. At x={x}: {d1:.2f}m → {d2:.2f}m (Δ={abs(d2-d1):.2f}m)")
    
    print("\n Vertical (middle column):")
    for i, (y, d1, d2) in enumerate(scan_results["vertical_transitions"]):
        print(f"  {i+1}. At y={y}: {d1:.2f}m → {d2:.2f}m (Δ={abs(d2-d1):.2f}m)")
    
    print("\n Diagonal (top-left to bottom-right):")
    for i, (pos, d1, d2) in enumerate(scan_results["diagonal1_transitions"]):
        print(f"  {i+1}. At pos={pos}: {d1:.2f}m → {d2:.2f}m (Δ={abs(d2-d1):.2f}m)")
    
    # Print object clusters
    print("\n === DETECTED OBJECTS ===")
    
    if not scan_results["clusters"]:
        print(" No objects detected")
    else:
        for i, cluster in enumerate(scan_results["clusters"]):
            avg_x, avg_y = cluster["center"]
            # Calculate angle from center
            angle = math.atan2(avg_x - 320, 240 - avg_y) * 180 / math.pi
            
            print(f"  {i+1}. Object at ({int(avg_x)}, {int(avg_y)}) - " +
                  f"distance: {cluster['depth']:.2f}m, angle: {angle:.1f}°, " +
                  f"confidence: {cluster['points']}")
    
    print("\nPress 'q' to quit, '+/-' to adjust max depth, '[/]' to adjust min depth")

def create_ascii_depth_map(depth_frame, width=80, height=24):
    """Create a text-based heatmap of the depth data using ASCII characters"""
    # Get depth data
    depth_data = np.asanyarray(depth_frame.get_data())
    
    # Convert to meters
    depth_meters = depth_data.astype(float) / 1000.0
    
    # Replace zeros (invalid measurements) with NaN
    depth_meters[depth_meters == 0] = np.nan
    
    # Create empty text map
    ascii_map = []
    
    # Characters for depth visualization (from closest to farthest)
    # Using characters with increasing "density"
    chars = " .'`^\",:;Il!i><~+_-?][}{1)(|/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$"
    
    # Calculate resize factors
    original_height, original_width = depth_meters.shape
    h_factor = original_height / height
    w_factor = original_width / width
    
    # Generate ASCII art
    for y in range(height):
        line = []
        for x in range(width):
            # Map to original image coordinates
            orig_x = int(x * w_factor)
            orig_y = int(y * h_factor)
            
            # Get depth value
            depth = depth_meters[orig_y, orig_x]
            
            # Check if valid depth
            if np.isnan(depth) or depth < MIN_DEPTH or depth > MAX_DEPTH:
                line.append(" ")  # No valid data
            else:
                # Map depth to character
                # Normalize depth to 0-1 range based on min/max depth
                norm_depth = (depth - MIN_DEPTH) / (MAX_DEPTH - MIN_DEPTH)
                # Invert so closer is "denser"
                norm_depth = 1.0 - norm_depth
                # Map to character index
                char_idx = int(norm_depth * (len(chars) - 1))
                line.append(chars[char_idx])
        
        ascii_map.append(''.join(line))
    
    return ascii_map

def create_color_depth_text_map(depth_frame, width=80, height=24):
    """Create a colored text-based heatmap of the depth data"""
    # Get depth data
    depth_data = np.asanyarray(depth_frame.get_data())
    
    # Convert to meters
    depth_meters = depth_data.astype(float) / 1000.0
    
    # Replace zeros (invalid measurements) with NaN
    depth_meters[depth_meters == 0] = np.nan
    
    # Calculate resize factors
    original_height, original_width = depth_meters.shape
    h_factor = original_height / height
    w_factor = original_width / width
    
    # Color map (distance in meters to ANSI color code)
    # Blue for far, red for near
    depth_map = []
    
    for y in range(height):
        line = []
        for x in range(width):
            # Map to original image coordinates
            orig_x = int(x * w_factor)
            orig_y = int(y * h_factor)
            
            # Get depth value
            depth = depth_meters[orig_y, orig_x]
            
            # Check if valid depth
            if np.isnan(depth) or depth < MIN_DEPTH or depth > MAX_DEPTH:
                line.append(" ")  # No valid data
            else:
                # Use block character for consistent display
                line.append("█")
        
        depth_map.append(''.join(line))
    
    return depth_map

def display_enhanced_text_view(depth_frame, regions_data, scan_results):
    """Display depth data in enhanced text format with heatmap"""
    os.system('cls' if os.name == 'nt' else 'clear')
    
    # Create ASCII depth map
    ascii_map = create_ascii_depth_map(depth_frame, width=80, height=20)
    
    # Print header
    print("\n === DEPTH DATA ANALYSIS ===")
    print(f" Range: {MIN_DEPTH}m - {MAX_DEPTH}m")
    
    # Print ASCII depth map
    print("\n === DEPTH HEATMAP ===")
    print(" (Closer objects appear as denser characters)")
    print(" " + "-" * 80)
    for line in ascii_map:
        print(" |" + line + "|")
    print(" " + "-" * 80)
    print(" Distance key: █=NEAR " + "".join(["▓", "▒", "░"]) + "=FAR")
    
    # Print grid analysis header
    print("\n === GRID ANALYSIS (8x8) ===")
    
    # Print grid headers
    print("    ", end="")
    for x in range(8):
        print(f"   {x}   ", end="")
    print("\n   +", end="")
    for x in range(8):
        print("-------", end="")
    print("+")
    
    # Print grid data
    for y in range(8):
        print(f" {y} |", end="")
        for x in range(8):
            region = next(r for r in regions_data if r["position"] == (x, y))
            
            if region["avg_depth"] > 0:
                print(f" {region['avg_depth']:.2f}m ", end="")
            else:
                print("   -   ", end="")
        print("|")
    
    print("   +", end="")
    for x in range(8):
        print("-------", end="")
    print("+")
    
    # Print detected transition points
    print("\n === DEPTH TRANSITIONS ===")
    print(" Horizontal (middle row):")
    h_transitions = scan_results["horizontal_transitions"]
    if not h_transitions:
        print("  No significant transitions detected")
    else:
        for i, (x, d1, d2) in enumerate(h_transitions[:5]):  # Show at most 5
            print(f"  {i+1}. At x={x}: {d1:.2f}m → {d2:.2f}m (Δ={abs(d2-d1):.2f}m)")
        if len(h_transitions) > 5:
            print(f"  ... and {len(h_transitions) - 5} more transitions")
    
    print("\n Vertical (middle column):")
    v_transitions = scan_results["vertical_transitions"]
    if not v_transitions:
        print("  No significant transitions detected")
    else:
        for i, (y, d1, d2) in enumerate(v_transitions[:5]):  # Show at most 5
            print(f"  {i+1}. At y={y}: {d1:.2f}m → {d2:.2f}m (Δ={abs(d2-d1):.2f}m)")
        if len(v_transitions) > 5:
            print(f"  ... and {len(v_transitions) - 5} more transitions")
    
    # Print object clusters
    print("\n === DETECTED OBJECTS ===")
    
    if not scan_results["clusters"]:
        print(" No objects detected")
    else:
        for i, cluster in enumerate(scan_results["clusters"]):
            avg_x, avg_y = cluster["center"]
            
            # Update angle calculation with correct FOV if not already present
            if "angle" not in cluster:
                # Calculate angle from center with 66 degree FOV
                hfov = 66
                normalized_pos = (avg_x - 320) / 320
                angle = normalized_pos * (hfov/2)
            else:
                angle = cluster["angle"]
            
            print(f"  {i+1}. Object at ({int(avg_x)}, {int(avg_y)}) - " +
                  f"distance: {cluster['depth']:.2f}m, angle: {angle:.1f}°, " +
                  f"confidence: {cluster['points']}")
    
    print("\nPress 'q' to quit, '+/-' to adjust max depth, '[/]' to adjust min depth")

def detect_simple_obstacles(depth_frame, width=640, height=480):
    """
    Detect obstacles and walls using only depth data patterns
    This function doesn't rely on OpenCV computer vision algorithms
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
    
    # =================== DETECTION STRATEGY ===================
    # 1. For walls: Look for large, consistent depth areas with some tolerance
    # 2. For obstacles: Look for larger, isolated objects with depth difference
    
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
    # More flexible wall detection with increased tolerance for depth variations
    wall_depth_tolerance = 0.25  # Allow up to 25cm variation for walls (was 0.15)
    min_wall_span = 4           # Require at least 4 grid cells (80px) for wall detection (was 5)
    
    # Find horizontal walls by looking for rows with relatively consistent depths
    for y in range(grid_height):
        row_depths = depth_grid[y, :]
        valid_in_row = valid_grid[y, :]
        
        if np.sum(valid_in_row) < grid_width // 4:  # Only need 25% of row to be valid (was 1/3)
            continue
            
        valid_depths = row_depths[valid_in_row]
        median_depth = np.median(valid_depths)
        
        # Check for consistent depth across the row (wall-like feature)
        consistent_points = 0
        for x in range(grid_width):
            if valid_grid[y, x] and abs(depth_grid[y, x] - median_depth) < wall_depth_tolerance:
                consistent_points += 1
        
        # If >50% of valid points in row have consistent depth, it's a wall (was 60%)
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
                walls.append({
                    "type": "horizontal_wall",
                    "position": (0, y_center),  # x, y center position
                    "distance": median_depth,
                    "width": wall_width,
                    "height": wall_height
                })
    
    # Find vertical walls similarly with columns
    for x in range(grid_width):
        col_depths = depth_grid[:, x]
        valid_in_col = valid_grid[:, x]
        
        if np.sum(valid_in_col) < grid_height // 4:  # Only need 25% of column to be valid
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
            wall_width = grid_size * 2  # Make vertical walls thicker for visualization
            
            # Check if this is close to an existing vertical wall
            duplicate = False
            for existing_wall in walls:
                if (existing_wall["type"] == "vertical_wall" and 
                    abs(existing_wall["position"][0] - x_center) < grid_size and
                    abs(existing_wall["distance"] - median_depth) < wall_depth_tolerance):
                    duplicate = True
                    break
                    
            if not duplicate:
                walls.append({
                    "type": "vertical_wall",
                    "position": (x_center, 0),  # x, y center position
                    "distance": median_depth,
                    "width": wall_width,
                    "height": wall_height
                })
    
    # OBSTACLE DETECTION 
    # Improved obstacle detection for larger objects
    # Obstacles need to be significantly closer than surroundings
    obstacle_depth_threshold = 0.3  # Objects must be at least 30cm closer than surroundings (was 0.2)
    min_neighbors_checked = 3      # Need at least this many valid neighbor cells to compare against
    
    # First pass: Find depth discontinuities
    for y in range(1, grid_height - 1):
        for x in range(1, grid_width - 1):
            if not valid_grid[y, x]:
                continue
                
            current_depth = depth_grid[y, x]
            
            # Check a wider area around the point (more neighbors)
            neighbor_coords = []
            for ny in range(y-2, y+3):  # Check 5x5 grid instead of 3x3
                for nx in range(x-2, x+3):
                    # Skip the center point itself
                    if ny == y and nx == x:
                        continue
                    neighbor_coords.append((ny, nx))
            
            depth_diffs = []
            valid_neighbors = 0
            
            for ny, nx in neighbor_coords:
                if 0 <= ny < grid_height and 0 <= nx < grid_width and valid_grid[ny, nx]:
                    depth_diffs.append(depth_grid[ny, nx] - current_depth)
                    valid_neighbors += 1
            
            # Need enough valid neighbors for a good comparison
            if valid_neighbors < min_neighbors_checked:
                continue
                
            # If this point is SIGNIFICANTLY closer than its surroundings
            if np.median(depth_diffs) > obstacle_depth_threshold:
                # Convert to image coordinates
                center_x = x * grid_size + grid_size // 2
                center_y = y * grid_size + grid_size // 2
                
                # Calculate angle from center with 66 degree FOV
                hfov = 66
                normalized_pos = (center_x - width/2) / (width/2)
                angle = normalized_pos * (hfov/2)
                
                # Make obstacles larger for better visibility
                obstacle_size = grid_size * 2
                
                obstacles.append({
                    "type": "obstacle",
                    "position": (center_x, center_y),
                    "distance": current_depth,
                    "angle": angle,
                    "size": obstacle_size,
                    "confidence": np.median(depth_diffs) / obstacle_depth_threshold  # Higher confidence when depth diff is larger
                })
    
    # Second pass: Merge nearby obstacles with more aggressive merging
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
        merge_space_threshold = grid_size * 3   # 3x grid cells distance for merging (was 2)
        merge_depth_threshold = 0.25           # 25cm depth difference allowed (was 0.15)
        
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
            # Make obstacles much larger for better visualization
            size = int(max(40, math.sqrt(len(merged)) * grid_size * 1.5))  # Minimum size of 40px, scale up by 1.5
            
            # Calculate angle from center with 66 degree FOV
            hfov = 66
            normalized_pos = (avg_x - width/2) / (width/2)
            angle = normalized_pos * (hfov/2)
            
            merged_obstacles.append({
                "type": "obstacle",
                "position": (int(avg_x), int(avg_y)),
                "distance": avg_dist,
                "angle": angle,
                "size": size,
                "confidence": sum(o.get("confidence", 1.0) for o in merged)  # Sum confidences
            })
    
    # Remove any obstacles that are too close to walls (likely false positives)
    filtered_obstacles = []
    wall_obstacle_distance = 1.0  # Min distance between obstacle and wall
    
    for obs in merged_obstacles:
        obs_x, obs_y = obs["position"]
        obs_dist = obs["distance"]
        is_near_wall = False
        
        for wall in walls:
            wall_x, wall_y = wall["position"]
            wall_dist = wall["distance"]
            
            # For horizontal walls
            if wall["type"] == "horizontal_wall":
                if (abs(wall_y - obs_y) < grid_size * 2 and 
                    abs(wall_dist - obs_dist) < wall_obstacle_distance):
                    is_near_wall = True
                    break
            # For vertical walls
            else:
                if (abs(wall_x - obs_x) < grid_size * 2 and 
                    abs(wall_dist - obs_dist) < wall_obstacle_distance):
                    is_near_wall = True
                    break
        
        if not is_near_wall:
            filtered_obstacles.append(obs)
    
    return {
        "obstacles": filtered_obstacles,
        "walls": walls
    }

def display_simple_detection(depth_frame, detection_results):
    """Display a simple ASCII representation of obstacle detection"""
    # Create ASCII depth map base
    ascii_map = create_ascii_depth_map(depth_frame, width=80, height=24)
    
    # Convert to list of lists for easier modification
    char_grid = [list(line) for line in ascii_map]
    
    # Define constants for this function
    image_width = 640   # Original image width
    image_height = 480  # Original image height
    grid_size = 20      # Grid size used in detection
    
    # Mark detected walls
    for wall in detection_results["walls"]:
        # Simplistic mapping from image coordinates to ASCII grid
        x, y = wall["position"]
        grid_x = int(x * 80 / image_width)
        grid_y = int(y * 24 / image_height)
        
        # Mark the wall with '=' for horizontal, '|' for vertical
        wall_char = '=' if wall["type"] == "horizontal_wall" else '|'
        
        # Determine width/height in grid coordinates
        wall_width = max(1, int(wall["width"] * 80 / image_width))
        wall_height = max(1, int(wall["height"] * 24 / image_height))
        
        if wall["type"] == "horizontal_wall":
            for i in range(max(0, grid_x), min(80, grid_x + wall_width)):
                if 0 <= grid_y < 24:
                    char_grid[grid_y][i] = wall_char
        else:  # vertical wall
            for i in range(max(0, grid_y), min(24, grid_y + wall_height)):
                if 0 <= grid_x < 80:
                    char_grid[i][grid_x] = wall_char
    
    # Mark detected obstacles
    for obstacle in detection_results["obstacles"]:
        x, y = obstacle["position"]
        grid_x = int(x * 80 / image_width)
        grid_y = int(y * 24 / image_height)
        
        # Mark the obstacle with 'O'
        if 0 <= grid_y < 24 and 0 <= grid_x < 80:
            char_grid[grid_y][grid_x] = 'O'
            
            # Add size indicator
            obstacle_size = obstacle.get("size", grid_size)  # Use obstacle size or default
            distance = obstacle["distance"]
            
            # Add distance info near the obstacle
            dist_text = f"{distance:.1f}m"
            for i, c in enumerate(dist_text):
                if grid_x + i + 1 < 80:
                    char_grid[grid_y][grid_x + i + 1] = c
    
    # Convert back to strings
    marked_map = [''.join(row) for row in char_grid]
    
    # Display the map
    os.system('cls' if os.name == 'nt' else 'clear')
    print("\n === SIMPLE OBSTACLE DETECTION ===")
    print(f" Range: {MIN_DEPTH}m - {MAX_DEPTH}m")
    print(" (O = obstacle, = = horizontal wall, | = vertical wall)\n")
    
    print(" " + "-" * 80)
    for line in marked_map:
        print(" |" + line + "|")
    print(" " + "-" * 80)
    
    # Print text summary
    print("\n === DETECTION SUMMARY ===")
    
    print("\n WALLS:")
    if not detection_results["walls"]:
        print("  No walls detected")
    else:
        for i, wall in enumerate(detection_results["walls"]):
            wtype = "Horizontal" if wall["type"] == "horizontal_wall" else "Vertical"
            
            # Calculate angle for walls if not already present
            if "angle" not in wall:
                x, _ = wall["position"]
                hfov = 66  # 66-degree FOV
                normalized_pos = (x - 320) / 320
                wall_angle = normalized_pos * (hfov/2)
            else:
                wall_angle = wall.get("angle", 0)
                
            print(f"  {i+1}. {wtype} wall at distance {wall['distance']:.2f}m, angle: {wall_angle:.1f}°")
    
    print("\n OBSTACLES:")
    if not detection_results["obstacles"]:
        print("  No obstacles detected")
    else:
        for i, obs in enumerate(detection_results["obstacles"]):
            name_str = f"{obs.get('name', 'Object')} " if 'name' in obs else ""
            id_str = f"(ID:{obs.get('id', '-')}) " if 'id' in obs else ""
            status_str = f"[{obs.get('status', 'New')}] " if 'status' in obs else ""
            
            print(f"  {i+1}. {name_str}{id_str}{status_str}at ({obs['position'][0]}, {obs['position'][1]}), " +
                  f"distance: {obs['distance']:.2f}m, angle: {obs['angle']:.1f}°")
    
    print("\nPress 'q' to quit, '+/-' to adjust max depth, '[/]' to adjust min depth")
    print("Press 'd' to toggle between detection modes, 't' to toggle tracking")

def create_top_down_view(width, height, obstructions, max_distance=5.0):
    """Create a top-down view visualization of obstacles and walls"""
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
    
    # Draw obstructions
    for obs in obstructions:
        distance = obs["distance"]
        obs_type = obs["type"]
        
        # Calculate angle if not present (especially for walls)
        if "angle" not in obs:
            # For walls, calculate angle based on position
            if obs_type in ["horizontal_wall", "vertical_wall"]:
                x, _ = obs["position"]
                # Calculate angle from center with 66 degree FOV
                hfov = 66  # 66-degree FOV
                normalized_pos = (x - 320) / 320
                angle = normalized_pos * (hfov/2)
            else:
                # Default to 0 for unknown types
                angle = 0
        else:
            angle = obs["angle"]
        
        if distance > 0 and distance <= max_distance:
            # Convert to cartesian coordinates
            rad = math.radians(angle)  # Use angle from calculated field of view
            x = int(center_x + distance * (height - 40) / max_distance * math.sin(rad))
            y = int(center_y - distance * (height - 40) / max_distance * math.cos(rad))
            
            # Draw different markers for vertical vs horizontal
            if obs_type == "obstacle":
                # Use object name or ID for color differentiation
                obj_id = obs.get("id", 0)
                obj_status = obs.get("status", "New")
                
                # Generate a unique color based on object ID
                if obj_id > 0:
                    r = (obj_id * 123) % 255
                    g = (obj_id * 85) % 255
                    b = (obj_id * 37) % 255
                    color = (b, g, r)  # BGR format for OpenCV
                else:
                    color = (0, 0, 255)  # Default red
                
                # Different marker based on tracking status
                if obj_status == "Confirmed":
                    cv2.circle(top_view, (x, y), 7, color, -1)  # Filled circle
                elif obj_status == "Pending":
                    cv2.circle(top_view, (x, y), 7, color, 2)  # Outlined circle
                else:
                    cv2.rectangle(top_view, (x-5, y-5), (x+5, y+5), color, 1)  # Rectangle
                
                # Add label with name if available
                if "name" in obs:
                    cv2.putText(top_view, obs["name"], (x+7, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            elif obs_type == "horizontal_wall" or obs_type == "vertical_wall":
                # Draw a small line perpendicular to the angle for walls
                perp_rad = rad + math.pi/2
                line_length = 15
                x1 = int(x + line_length * math.sin(perp_rad))
                y1 = int(y - line_length * math.cos(perp_rad))
                x2 = int(x - line_length * math.sin(perp_rad))
                y2 = int(y - line_length * math.cos(perp_rad))
                cv2.line(top_view, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue line
            
            # Add distance label
            cv2.putText(top_view, f"{distance:.1f}m", (x+5, y-5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    return top_view

# Add new class for persistent object tracking
class ObstacleTracker:
    """Tracks obstacles across multiple frames to maintain identity and history"""
    
    def __init__(self, max_age=30, min_hits=3, distance_threshold=0.5, angle_threshold=10):
        """
        Initialize obstacle tracker
        
        Parameters:
        - max_age: Maximum number of frames an object can be missing before removing
        - min_hits: Minimum number of detections needed to confirm an object
        - distance_threshold: Maximum distance difference (meters) for matching objects
        - angle_threshold: Maximum angle difference (degrees) for matching objects
        """
        self.obstacles = {}  # Dict of tracked obstacles, key = ID
        self.next_id = 1     # Next available obstacle ID
        self.max_age = max_age
        self.min_hits = min_hits
        self.distance_threshold = distance_threshold
        self.angle_threshold = angle_threshold
        
        # Maintain a history of object positions for smoothing
        self.history = {}
        
        # Names for common obstacles
        self.common_names = [
            "Chair", "Table", "Box", "Plant", "Shelf", 
            "Bucket", "Cart", "Bag", "Tree", "Doorway"
        ]
        self.assigned_names = {}  # Map IDs to names
    
    def update(self, detected_obstacles):
        """
        Update tracked obstacles with new detections
        
        Parameters:
        - detected_obstacles: List of obstacle dictionaries from detection function
        
        Returns:
        - Dictionary of tracked obstacles with additional tracking info
        """
        # Predict new locations based on previous tracking (if implemented)
        
        # Match detections to existing tracks
        matches, unmatched_detections, unmatched_trackers = self._match_obstacles(detected_obstacles)
        
        # Update matched obstacles
        for track_id, detection_idx in matches:
            obs = detected_obstacles[detection_idx].copy()
            self.obstacles[track_id]["position"] = obs["position"]
            self.obstacles[track_id]["distance"] = obs["distance"]
            self.obstacles[track_id]["angle"] = obs["angle"]
            self.obstacles[track_id]["size"] = obs["size"]
            self.obstacles[track_id]["confidence"] = obs.get("confidence", 1.0)
            self.obstacles[track_id]["type"] = obs["type"]
            self.obstacles[track_id]["last_seen"] = 0
            self.obstacles[track_id]["hits"] += 1
            
            # Update position history for smoother tracking
            if track_id not in self.history:
                self.history[track_id] = []
            
            self.history[track_id].append({
                "position": obs["position"], 
                "distance": obs["distance"],
                "angle": obs["angle"]
            })
            
            # Keep only the last 5 positions
            if len(self.history[track_id]) > 5:
                self.history[track_id].pop(0)
        
        # Create new tracks for unmatched detections
        for i in unmatched_detections:
            obs = detected_obstacles[i].copy()
            new_id = self.next_id
            self.next_id += 1
            
            self.obstacles[new_id] = obs.copy()
            self.obstacles[new_id]["id"] = new_id
            self.obstacles[new_id]["last_seen"] = 0
            self.obstacles[new_id]["hits"] = 1
            self.obstacles[new_id]["track_status"] = "New"
            
            # Assign a name to the new obstacle
            self._assign_name(new_id)
            
            # Initialize position history
            self.history[new_id] = [{
                "position": obs["position"], 
                "distance": obs["distance"],
                "angle": obs["angle"]
            }]
        
        # Update unmatched existing tracks (missing obstacles)
        for track_id in unmatched_trackers:
            self.obstacles[track_id]["last_seen"] += 1
            self.obstacles[track_id]["track_status"] = "Missing"
        
        # Remove outdated obstacles
        track_ids = list(self.obstacles.keys())
        for track_id in track_ids:
            # If obstacle hasn't been seen for too long and was confirmed
            if (self.obstacles[track_id]["last_seen"] > self.max_age and
                self.obstacles[track_id]["hits"] >= self.min_hits):
                self.obstacles.pop(track_id, None)
                self.history.pop(track_id, None)
            # If obstacle had very few hits and disappeared
            elif (self.obstacles[track_id]["last_seen"] > self.min_hits and
                  self.obstacles[track_id]["hits"] < self.min_hits):
                self.obstacles.pop(track_id, None)
                self.history.pop(track_id, None)
        
        # Create smoothed output with stabilized positions
        smoothed_obstacles = []
        for track_id, obs in self.obstacles.items():
            # Only include confirmed obstacles
            if obs["hits"] >= self.min_hits:
                smoothed_obs = obs.copy()
                
                # Apply smoothing if we have history
                if track_id in self.history and len(self.history[track_id]) > 1:
                    history = self.history[track_id]
                    
                    # Calculate weighted average of recent positions (more weight to recent)
                    weights = [i+1 for i in range(len(history))]
                    total_weight = sum(weights)
                    
                    avg_x = sum(h["position"][0] * w for h, w in zip(history, weights)) / total_weight
                    avg_y = sum(h["position"][1] * w for h, w in zip(history, weights)) / total_weight
                    avg_dist = sum(h["distance"] * w for h, w in zip(history, weights)) / total_weight
                    avg_angle = sum(h["angle"] * w for h, w in zip(history, weights)) / total_weight
                    
                    smoothed_obs["position"] = (int(avg_x), int(avg_y))
                    smoothed_obs["distance"] = avg_dist
                    smoothed_obs["angle"] = avg_angle
                    
                # Add name and status
                smoothed_obs["name"] = self.assigned_names.get(track_id, f"Object-{track_id}")
                smoothed_obs["status"] = "Confirmed" if obs["hits"] >= self.min_hits else "Pending"
                
                smoothed_obstacles.append(smoothed_obs)
        
        return smoothed_obstacles
    
    def _match_obstacles(self, detected_obstacles):
        """
        Match detected obstacles to existing tracked obstacles
        
        Uses distance, angle, and size to determine matches
        """
        matches = []
        unmatched_detections = list(range(len(detected_obstacles)))
        unmatched_trackers = list(self.obstacles.keys())
        
        if len(unmatched_detections) == 0 or len(unmatched_trackers) == 0:
            return matches, unmatched_detections, unmatched_trackers
        
        # Build cost matrix for matching
        cost_matrix = np.zeros((len(unmatched_trackers), len(unmatched_detections)))
        
        for i, track_id in enumerate(unmatched_trackers):
            for j, det_idx in enumerate(unmatched_detections):
                # Calculate distance between objects in 3D space
                track = self.obstacles[track_id]
                detection = detected_obstacles[det_idx]
                
                # Distance difference in real world (meters)
                dist_diff = abs(track["distance"] - detection["distance"])
                
                # Position difference (normalized by distance to account for perspective)
                # Objects that are further away can move more pixels between frames
                pos_diff = np.sqrt(
                    (track["position"][0] - detection["position"][0])**2 +
                    (track["position"][1] - detection["position"][1])**2
                ) / max(1.0, track["distance"] * 100)  # Scale by distance
                
                # Angle difference (for direction-based matching)
                angle_diff = abs(track["angle"] - detection["angle"])
                
                # Weight these factors to create a cost
                cost = dist_diff * 0.6 + pos_diff * 0.3 + angle_diff * 0.1
                
                # Apply max cost for unlikely matches
                if (dist_diff > self.distance_threshold or
                    angle_diff > self.angle_threshold or
                    track["type"] != detection["type"]):  # Must be same type (wall/obstacle)
                    cost = 1000.0  # Very high cost to prevent matching
                
                cost_matrix[i][j] = cost
        
        # Use linear assignment to find optimal matches (greedy approach for simplicity)
        while True:
            if cost_matrix.size == 0:
                break
                
            # Find min cost and match
            i, j = np.unravel_index(np.argmin(cost_matrix, axis=None), cost_matrix.shape)
            
            if cost_matrix[i, j] > 10.0:  # Cost threshold for valid matches
                break
                
            # Add match
            track_id = unmatched_trackers[i]
            det_idx = unmatched_detections[j]
            matches.append((track_id, det_idx))
            
            # Remove matched elements
            unmatched_trackers.pop(i)
            unmatched_detections.pop(j)
            
            # Update cost matrix
            cost_matrix = np.delete(cost_matrix, i, axis=0)
            cost_matrix = np.delete(cost_matrix, j, axis=1)
            
            if cost_matrix.size == 0:
                break
        
        return matches, unmatched_detections, unmatched_trackers
    
    def _assign_name(self, object_id):
        """Assign a name to a tracked object"""
        # If all common names are used, use generic name with ID
        if len(self.assigned_names) >= len(self.common_names):
            self.assigned_names[object_id] = f"Object-{object_id}"
        else:
            # Find unused name
            used_names = set(self.assigned_names.values())
            for name in self.common_names:
                if name not in used_names:
                    self.assigned_names[object_id] = name
                    break

# Initialize the global tracker
obstacle_tracker = ObstacleTracker(max_age=30, min_hits=3)

def visualize_detections(color_image, depth_colormap, detection_results):
    """
    Create visual overlays showing detected walls and obstacles on color and depth images
    Returns annotated copies of the input images
    """
    # Create copies of the input images to avoid modifying originals
    color_viz = color_image.copy()
    depth_viz = depth_colormap.copy()
    
    # Define colors for different detection types
    wall_color = (255, 0, 0)      # Blue for walls
    obstacle_color = (0, 0, 255)  # Red for obstacles
    text_color = (255, 255, 255)  # White for text
    
    # Draw walls on both visualizations
    wall_count = 0
    for wall in detection_results["walls"]:
        wall_count += 1
        
        # Get wall position and dimensions
        x, y = wall["position"]
        width = wall["width"]
        height = wall["height"]
        distance = wall["distance"]
        
        # Calculate full rectangle coordinates based on wall type
        if wall["type"] == "horizontal_wall":
            # Horizontal walls span the width
            x1, y1 = 0, max(0, y - height//2)
            x2, y2 = color_image.shape[1], min(color_image.shape[0], y + height//2)
        else:  # vertical wall
            # Vertical walls span the height
            x1, y1 = max(0, x - width//2), 0
            x2, y2 = min(color_image.shape[1], x + width//2), color_image.shape[0]
        
        # Draw rectangles with semi-transparency
        overlay_color = color_viz.copy()
        overlay_depth = depth_viz.copy()
        
        cv2.rectangle(overlay_color, (x1, y1), (x2, y2), wall_color, -1)
        cv2.rectangle(overlay_depth, (x1, y1), (x2, y2), wall_color, -1)
        
        # Apply transparency
        alpha = 0.1  # Transparency factor
        cv2.addWeighted(overlay_color, alpha, color_viz, 1 - alpha, 0, color_viz)
        cv2.addWeighted(overlay_depth, alpha, depth_viz, 1 - alpha, 0, depth_viz)
        
        # Draw outline and label
        cv2.rectangle(color_viz, (x1, y1), (x2, y2), wall_color, 2)
        cv2.rectangle(depth_viz, (x1, y1), (x2, y2), wall_color, 2)
        
        wall_type = "Horizontal" if wall["type"] == "horizontal_wall" else "Vertical"
        label = f"{wall_type} Wall #{wall_count}: {distance:.2f}m"
        
        # Place label at appropriate position
        if wall["type"] == "horizontal_wall":
            label_x = 10
            label_y = y1 + 20
        else:
            label_x = x1 + 10
            label_y = 20
        
        cv2.putText(color_viz, label, (label_x, label_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
        cv2.putText(depth_viz, label, (label_x, label_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 2)
    
    # Draw obstacles - make them more prominent
    obstacle_count = 0
    for obstacle in detection_results["obstacles"]:
        obstacle_count += 1
        
        # Get obstacle position and properties
        x, y = obstacle["position"]
        distance = obstacle["distance"]
        angle = obstacle["angle"]
        size = obstacle.get("size", 40)  # Default size if not specified
        confidence = obstacle.get("confidence", 1.0)
        
        # Get object name and ID if available
        obstacle_name = obstacle.get("name", f"Obstacle {obstacle_count}")
        obstacle_id = obstacle.get("id", -1)
        obstacle_status = obstacle.get("status", "New")
        
        # Use different colors based on tracking status
        if obstacle_status == "Confirmed":
            obs_color = (0, 0, 255)  # Red for confirmed obstacles
        elif obstacle_status == "Pending":
            obs_color = (0, 165, 255)  # Orange for pending
        else:
            obs_color = (0, 255, 255)  # Yellow for new
        
        # Calculate radius based on size - making obstacles larger and more visible
        radius = max(10, size // 3)  # Minimum radius of 10px
        
        # Draw filled circle with semi-transparency
        overlay_color = color_viz.copy()
        overlay_depth = depth_viz.copy()
        
        cv2.circle(overlay_color, (x, y), radius, obs_color, -1)
        cv2.circle(overlay_depth, (x, y), radius, obs_color, -1)
        
        # Apply transparency - more opaque for higher confidence
        alpha = min(0.7, 0.3 + (confidence * 0.1))  # Scale opacity with confidence
        cv2.addWeighted(overlay_color, alpha, color_viz, 1 - alpha, 0, color_viz)
        cv2.addWeighted(overlay_depth, alpha, depth_viz, 1 - alpha, 0, depth_viz)
        
        # Draw outline
        cv2.circle(color_viz, (x, y), radius, obs_color, 2)
        cv2.circle(depth_viz, (x, y), radius, obs_color, 2)
        
        # Draw direction line (based on angle)
        line_length = max(50, radius * 2)  # Scale line with obstacle size
        end_x = int(x + line_length * math.sin(math.radians(angle)))
        end_y = int(y - line_length * math.cos(math.radians(angle)))
        
        cv2.line(color_viz, (x, y), (end_x, end_y), (0, 255, 255), 2)
        cv2.line(depth_viz, (x, y), (end_x, end_y), (0, 255, 255), 2)
        
        # Add label with name, distance and angle
        label = f"{obstacle_name}: {distance:.2f}m {angle:.0f}°"
        
        # Position text relative to obstacle size
        text_x = x + radius + 5
        text_y = y
        
        cv2.putText(color_viz, label, (text_x, text_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        cv2.putText(depth_viz, label, (text_x, text_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        
        # Add ID and status on second line if tracked
        if obstacle_id >= 0:
            status_text = f"ID:{obstacle_id} ({obstacle_status})"
            cv2.putText(color_viz, status_text, (text_x, text_y + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
            cv2.putText(depth_viz, status_text, (text_x, text_y + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
    
    # Add summary at the top
    summary = f"Walls: {len(detection_results['walls'])}  Obstacles: {len(detection_results['obstacles'])}"
    cv2.putText(color_viz, summary, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(depth_viz, summary, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    return color_viz, depth_viz

def main():
    global MIN_DEPTH, MAX_DEPTH, obstacle_tracker
    
    # Initialize camera
    try:
        pipeline, align = initialize_camera()
        
        print("Starting depth analysis...")
        print("Press any key to start text view...")
        cv2.waitKey(0)
        
        # Detection mode (0: detailed analysis, 1: simple obstacle detection)
        detection_mode = 1
        
        # Visualization flag
        show_detections = True
        
        # Tracking flag
        enable_tracking = True
        
        # Add a new flag for filtering aggressiveness
        filter_strength = 1  # 0=minimal, 1=medium, 2=strong filtering
        
        while True:
            # Get frames
            depth_frame, color_frame = get_frames(pipeline, align)
            if depth_frame is None:
                continue
            
            # Convert frames to images
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Create a normalized depth image for display
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            if detection_mode == 0:    
                # Analyze depth in regions
                regions_data = analyze_depth_regions(depth_frame)
                
                # Scan for objects and transitions
                scan_results = scan_for_objects(depth_frame)
                
                # Display enhanced text representation with heatmap
                display_enhanced_text_view(depth_frame, regions_data, scan_results)
                
                # No visualization for detailed analysis mode
                viz_color = color_image
                viz_depth = depth_colormap
                
            else:
                # Perform simple obstacle detection
                detection_results = detect_simple_obstacles(depth_frame)
                
                # Apply obstacle tracking if enabled
                if enable_tracking:
                    tracked_obstacles = obstacle_tracker.update(detection_results["obstacles"])
                    detection_results["obstacles"] = tracked_obstacles
                
                # Display simple detection view
                display_simple_detection(depth_frame, detection_results)
                
                # Create top-down view
                if detection_results["obstacles"] or detection_results["walls"]:
                    top_down_size = 240
                    
                    # Prepare walls with angle information for top-down view
                    walls_with_angle = []
                    for wall in detection_results["walls"]:
                        wall_copy = wall.copy()
                        # Add angle information if missing
                        if "angle" not in wall_copy:
                            x, _ = wall_copy["position"]
                            hfov = 66  # 66-degree FOV
                            normalized_pos = (x - 320) / 320
                            wall_copy["angle"] = normalized_pos * (hfov/2)
                        walls_with_angle.append(wall_copy)
                    
                    # Combine obstacles and walls with complete information
                    all_objects = detection_results["obstacles"] + walls_with_angle
                    
                    top_down = create_top_down_view(
                        top_down_size, top_down_size, 
                        all_objects, 
                        MAX_DEPTH
                    )
                    
                    # Show top-down view in separate window
                    cv2.namedWindow('Top-Down View', cv2.WINDOW_NORMAL)
                    cv2.imshow('Top-Down View', top_down)
                
                # Create visualization with detection overlays
                if show_detections:
                    viz_color, viz_depth = visualize_detections(
                        color_image, depth_colormap, detection_results)
                else:
                    viz_color = color_image
                    viz_depth = depth_colormap
            
            # Show visualizations
            cv2.namedWindow('Color View', cv2.WINDOW_NORMAL)
            cv2.imshow('Color View', viz_color)
            
            cv2.namedWindow('Depth View', cv2.WINDOW_NORMAL)
            cv2.imshow('Depth View', viz_depth)
            
            # Handle keyboard input
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('d'):
                detection_mode = 1 - detection_mode
                print(f"Switched to {'simple obstacle detection' if detection_mode == 1 else 'detailed analysis'} mode")
            elif key == ord('v'):
                show_detections = not show_detections
                print(f"Detection visualization turned {'on' if show_detections else 'off'}")
            elif key == ord('t'):
                enable_tracking = not enable_tracking
                if not enable_tracking:
                    # Reset tracker when disabling to start fresh when re-enabled
                    obstacle_tracker = ObstacleTracker(max_age=30, min_hits=3)
                print(f"Object tracking turned {'on' if enable_tracking else 'off'}")
            elif key == ord('r'):
                # Reset tracker
                obstacle_tracker = ObstacleTracker(max_age=30, min_hits=3)
                print("Object tracker reset")
            elif key == ord('f'):
                filter_strength = (filter_strength + 1) % 3
                print(f"Filter strength set to {filter_strength} ({'minimal' if filter_strength == 0 else 'medium' if filter_strength == 1 else 'strong'})")
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
            
            time.sleep(0.1)  # Small delay to prevent high CPU usage
            
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
        
    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Application closed")

if __name__ == "__main__":
    main()
