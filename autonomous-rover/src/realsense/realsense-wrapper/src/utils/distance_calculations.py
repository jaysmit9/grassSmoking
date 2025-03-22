def convert_raw_depth_to_meters(raw_depth):
    """
    Convert raw depth value to meters.
    
    Args:
        raw_depth: Raw depth value from RealSense camera
        
    Returns:
        Distance in meters
    """
    # RealSense depth data is already in meters
    return raw_depth

def calculate_average_distance(depth_frame):
    width = depth_frame.width
    height = depth_frame.height
    total_distance = 0
    count = 0

    for y in range(height):
        for x in range(width):
            raw_depth = depth_frame.get_distance(x, y)
            if raw_depth > 0:  # Only consider valid depth values
                total_distance += convert_raw_depth_to_meters(raw_depth)
                count += 1

    return total_distance / count if count > 0 else 0

def get_distance_at_pixel(depth_frame, x, y):
    raw_depth = depth_frame.get_distance(x, y)
    return convert_raw_depth_to_meters(raw_depth) if raw_depth > 0 else float('inf')