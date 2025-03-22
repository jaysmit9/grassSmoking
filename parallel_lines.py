import numpy as np
import matplotlib.pyplot as plt
import json
from matplotlib.patches import Polygon
from geopy.distance import geodesic
from shapely.geometry import LineString, Polygon as ShapelyPolygon
import matplotlib.colors as mcolors

def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate the bearing from point 1 to point 2"""
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    y = np.sin(lon2 - lon1) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(lon2 - lon1)
    bearing = np.arctan2(y, x)
    return np.degrees(bearing)

def haversine_distance(coord1, coord2):
    """Calculate distance between two GPS coordinates"""
    return geodesic(coord1, coord2).meters

def generate_parallel_lines(polygon_points, spacing=3.0):
    """Generate parallel lines inside a polygon based on first edge"""
    # Extract the first two points to define the reference line
    p1 = (polygon_points[1]['lat'], polygon_points[1]['lon'])
    p2 = (polygon_points[2]['lat'], polygon_points[2]['lon'])
    
    # Calculate the bearing of the first edge
    bearing = calculate_bearing(p1[0], p1[1], p2[0], p2[1])
    perpendicular_bearing = (bearing - 90) % 360
    
    # Calculate meter to degree conversions (approximate)
    lat_avg = (p1[0] + p2[0]) / 2
    meters_per_lat = 111320  # Approximate meters per degree latitude
    meters_per_lon = 111320 * np.cos(np.radians(lat_avg))  # Approximate meters per degree longitude
    
    # Calculate the shift vector for spacing
    dx = spacing * np.sin(np.radians(perpendicular_bearing)) / meters_per_lat
    dy = spacing * np.cos(np.radians(perpendicular_bearing)) / meters_per_lon
    
    # Create a Shapely polygon to check if lines are inside
    polygon_coords = [(p['lat'], p['lon']) for p in polygon_points]
    shapely_polygon = ShapelyPolygon(polygon_coords)
    
    # First, add the reference line along the first edge
    base_line = LineString([p1, p2])
    extended_base_line = extend_line_to_polygon(base_line, shapely_polygon)
    
    # Calculate the parallel lines
    parallel_lines = []
    if extended_base_line:
        parallel_lines.append(extended_base_line)
    
    current_p1 = p1
    current_p2 = p2
    
    # Initial check if the correct direction is inward
    test_point = (current_p1[0] + dx, current_p1[1] + dy)
    test_line = LineString([test_point, (current_p2[0] + dx, current_p2[1] + dy)])
    
    # If the test line is not inside the polygon, reverse the direction
    if not test_line.intersects(shapely_polygon):
        dx = -dx
        dy = -dy
        
    # Generate lines in both directions from the reference line
    directions = [1, -1]  # Try both directions if needed
    
    for direction in directions:
        current_p1 = p1
        current_p2 = p2
        
        # Generate lines while they remain inside the polygon
        max_iterations = 50  # Safety limit
        for i in range(max_iterations):
            # Calculate the next parallel line
            next_p1 = (current_p1[0] + direction * dx, current_p1[1] + direction * dy)
            next_p2 = (current_p2[0] + direction * dx, current_p2[1] + direction * dy)
            next_line = LineString([next_p1, next_p2])
            
            # Extend the line to cover the polygon
            extended_line = extend_line_to_polygon(next_line, shapely_polygon)
            if extended_line is None:
                break  # Line doesn't intersect polygon anymore
                
            parallel_lines.append(extended_line)
            
            # Update for next iteration
            current_p1 = next_p1
            current_p2 = next_p2
            
            # Check if the next line would still be inside the polygon
            test_point = (current_p1[0] + direction * dx, current_p1[1] + direction * dy)
            if not shapely_polygon.intersects(LineString([test_point, (current_p2[0] + direction * dx, current_p2[1] + direction * dy)])):
                break
    
    return parallel_lines

def extend_line_to_polygon(line, polygon, buffer_distance=1.0):
    """Extend a line to fully intersect with polygon boundaries, then shrink by buffer_distance"""
    if not line.intersects(polygon):
        return None
    
    # Get the line coordinates
    coords = list(line.coords)
    if len(coords) < 2:
        return None
    
    # Get the line's direction vector
    x1, y1 = coords[0]
    x2, y2 = coords[-1]
    direction_vector = (x2 - x1, y2 - y1)
    
    # Normalize the direction vector
    magnitude = np.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
    if magnitude > 0:
        normalized_vector = (direction_vector[0]/magnitude, direction_vector[1]/magnitude)
    else:
        return None
    
    # Create a very long line in both directions to ensure it extends beyond the polygon
    extension_factor = 1.0  # A value large enough in degree terms
    extended_start = (
        x1 - extension_factor * normalized_vector[0],
        y1 - extension_factor * normalized_vector[1]
    )
    extended_end = (
        x2 + extension_factor * normalized_vector[0],
        y2 + extension_factor * normalized_vector[1]
    )
    
    extended_line = LineString([extended_start, extended_end])
    
    # Find the intersection with the polygon boundary
    boundary = polygon.boundary
    intersection = extended_line.intersection(boundary)
    
    # Process the intersection
    if intersection.is_empty:
        return None
    elif intersection.geom_type == 'Point':
        return None  # Only one intersection point, not a line
    elif intersection.geom_type == 'MultiPoint':
        # Get the two farthest points for a complete crossing
        points = list(intersection.geoms)
        if len(points) >= 2:
            # Take the two farthest points to ensure we cross the entire polygon
            distances = []
            for i in range(len(points)):
                for j in range(i+1, len(points)):
                    dist = points[i].distance(points[j])
                    distances.append((dist, i, j))
            
            if distances:
                # Get the pair with maximum distance
                max_dist, idx1, idx2 = max(distances, key=lambda x: x[0])
                
                # Create a line connecting these points
                boundary_line = LineString([points[idx1], points[idx2]])
                
                # Now shrink this line by buffer_distance from each end
                return shrink_line(boundary_line, buffer_distance)
        return None
    elif intersection.geom_type == 'LineString':
        # Shrink the line
        return shrink_line(intersection, buffer_distance)
    elif intersection.geom_type == 'MultiLineString':
        # Get the longest segment
        if len(intersection.geoms) > 0:
            longest_segment = max(intersection.geoms, key=lambda x: x.length)
            return shrink_line(longest_segment, buffer_distance)
        return None
    else:
        return None

def shrink_line(line, buffer_distance=1.0):
    """Shrink a line by buffer_distance meters from each end"""
    if line is None or line.length == 0:
        return None
        
    coords = list(line.coords)
    if len(coords) < 2:
        return None
        
    # Get line endpoints
    start_point = coords[0]
    end_point = coords[-1]
    
    # Calculate the bearing (direction) of the line
    start_lat, start_lon = start_point
    end_lat, end_lon = end_point
    
    # Calculate the total length of the line in meters
    total_length = haversine_distance((start_lat, start_lon), (end_lat, end_lon))
    
    # If the line is too short to shrink, return None
    if total_length <= 2 * buffer_distance:
        return None
    
    # Calculate the bearing of the line
    forward_bearing = calculate_bearing(start_lat, start_lon, end_lat, end_lon)
    backward_bearing = (forward_bearing + 180) % 360
    
    # Calculate meter to degree conversions (approximate)
    lat_avg = (start_lat + end_lat) / 2
    meters_per_lat = 111320  # Approximate meters per degree latitude
    meters_per_lon = 111320 * np.cos(np.radians(lat_avg))  # Approximate meters per degree longitude
    
    # Calculate new start point (buffer_distance meters along the line from start)
    # Convert buffer distance to degrees
    lat_shift = buffer_distance * np.cos(np.radians(forward_bearing)) / meters_per_lat
    lon_shift = buffer_distance * np.sin(np.radians(forward_bearing)) / meters_per_lon
    new_start = (start_lat + lat_shift, start_lon + lon_shift)
    
    # Calculate new end point (buffer_distance meters inward from end)
    lat_shift = buffer_distance * np.cos(np.radians(backward_bearing)) / meters_per_lat
    lon_shift = buffer_distance * np.sin(np.radians(backward_bearing)) / meters_per_lon
    new_end = (end_lat + lat_shift, end_lon + lon_shift)
    
    # Create and return the shortened line
    return LineString([new_start, new_end])

# Define tunable parameters
ELLIPSE_CURVATURE = 3.0  # Controls how curved the ellipse is (higher = flatter)
NUM_ELLIPSE_POINTS = 20  # Number of points to generate for each ellipse
SETBACK_DISTANCE = 1.0  # Distance to back up from waypoint 1
MAX_CURVATURE = 10.0  # Maximum curvature value to try
CURVATURE_INCREMENT = 0.5  # Increment for curvature adjustment

def create_ellipse_path(start_point, end_point, polygon_data, curve_direction=1, num_points=NUM_ELLIPSE_POINTS, ellipse_curvature=ELLIPSE_CURVATURE):
    """
    Create an elliptical path from start_point to end_point, ensuring it stays within the polygon
    
    Args:
        start_point: (lat, lon) tuple for starting point
        end_point: (lat, lon) tuple for ending point
        polygon_data: List of polygon points
        curve_direction: 1 for one direction, -1 for the opposite
        num_points: Number of points to generate for the ellipse
        ellipse_curvature: Initial curvature value
        
    Returns:
        LineString of the elliptical path, or None if no valid path is found
    """
    # Create a Shapely polygon for checking containment
    polygon_coords = [(p['lat'], p['lon']) for p in polygon_data]
    shapely_polygon = ShapelyPolygon(polygon_coords)
    
    # Iterate to find a valid curvature
    current_curvature = ellipse_curvature
    
    while current_curvature <= MAX_CURVATURE:
        # Calculate direct path bearing and distance
        direct_distance = haversine_distance(start_point, end_point)
        direct_bearing = calculate_bearing(start_point[0], start_point[1], end_point[0], end_point[1])
        
        # Calculate midpoint (center of ellipse)
        mid_lat = (start_point[0] + end_point[0]) / 2
        mid_lon = (start_point[1] + end_point[1]) / 2
        
        # Calculate meter to degree conversions
        meters_per_lat = 111320  # Approximate meters per degree latitude
        meters_per_lon = 111320 * np.cos(np.radians(mid_lat))  # Approximate meters per degree longitude
        
        # Set semi-major and semi-minor axes
        a = direct_distance / 2  # Semi-major axis is half the distance
        b = curve_direction * direct_distance / current_curvature  # Semi-minor axis controls curvature
        
        # Generate elliptical path
        path_points = []
        
        for t in np.linspace(0, np.pi, num_points):
            # Standard parametric equation of an ellipse
            x_ellipse = a * np.cos(t)
            y_ellipse = b * np.sin(t)
            
            # Rotate the ellipse to align with bearing
            rotation_angle = np.radians(direct_bearing)
            x_rot = x_ellipse * np.cos(rotation_angle) - y_ellipse * np.sin(rotation_angle)
            y_rot = x_ellipse * np.sin(rotation_angle) + y_ellipse * np.cos(rotation_angle)
            
            # Convert to lat/lon changes
            lat_change = x_rot / meters_per_lat
            lon_change = y_rot / meters_per_lon
            
            # Add to midpoint coordinates
            point_lat = mid_lat + lat_change
            point_lon = mid_lon + lon_change
            
            path_points.append((point_lat, point_lon))
        
        # Create a LineString from the points
        ellipse_path = LineString(path_points)
        
        # Check if the ellipse path is within the polygon
        if ellipse_path.within(shapely_polygon):
            # If the ellipse is within the polygon, return it
            print(f"Found valid ellipse with curvature {current_curvature:.2f}")
            return ellipse_path
        
        # Increment curvature for next iteration
        current_curvature += CURVATURE_INCREMENT
    
    # If no valid ellipse is found, return None
    print("No valid ellipse found within curvature limits")
    return None

def visualize_polygon_and_lines(polygon_data, parallel_lines, show_ellipse=False, start_point=None, show_connections=False):
    """Visualize the polygon and parallel lines"""
    # Extract coordinates
    lats = [point['lat'] for point in polygon_data]
    lons = [point['lon'] for point in polygon_data]
    
    # Create figure
    plt.figure(figsize=(12, 10))
    
    # Plot polygon
    plt.plot(lats, lons, 'ko-', linewidth=2, label='Polygon Boundary')
    plt.fill(lats, lons, alpha=0.1, color='blue')
    
    # Plot first edge in a different color
    plt.plot([lats[0], lats[1]], [lons[0], lons[1]], 'ro-', linewidth=3, label='Reference Edge')
    
    # Plot ellipse path in a distinct color if present
    if show_ellipse and parallel_lines and len(parallel_lines) > 0:
        ellipse = parallel_lines[0]
        x, y = ellipse.xy
        plt.plot(x, y, color='green', linewidth=2, linestyle='-',
                 label='Elliptical Approach Path')
    
    # Plot parallel lines
    colors = list(mcolors.TABLEAU_COLORS.values())
    start_idx = 1 if show_ellipse else 0  # Skip ellipse path if present
    
    for i, line in enumerate(parallel_lines[start_idx:], start=start_idx):
        x, y = line.xy
        plt.plot(x, y, color=colors[i % len(colors)], linewidth=1.5, linestyle='--',
                 label=f'Parallel Line {i}' if i == start_idx else None)
    
    # Annotate points
    for i, (lat, lon) in enumerate(zip(lats, lons)):
        plt.annotate(f"P{i}", (lat, lon), fontsize=12)
    
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.title('Polygon with Parallel Lines (3m spacing) and Approach Path')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('polygon_parallel_lines_with_approach.png', dpi=300)
    plt.show()
    
    # Also create a more detailed visualization showing the spacing
    plt.figure(figsize=(12, 8))
    
    # Create a heatmap-like visualization of the coverage
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons)
    
    # Add spacing information
    first_line = parallel_lines[0] if parallel_lines else None
    if first_line:
        x1, y1 = first_line.xy
        dist_text = f"Line Spacing: 3.0 meters\nTotal Lines: {len(parallel_lines)}"
        plt.annotate(dist_text, (min_lat + (max_lat - min_lat)*0.05, 
                               min_lon + (max_lon - min_lon)*0.95),
                   bbox=dict(facecolor='white', alpha=0.7))
    
    # Show information about area
    area = ShapelyPolygon([(p['lat'], p['lon']) for p in polygon_data]).area
    # Approximate conversion from degrees² to m²
    area_approx = area * 111320 * 111320 * np.cos(np.radians((min_lat + max_lat)/2))
    plt.annotate(f"Polygon Area: ~{area_approx:.1f} m²", 
               (min_lat + (max_lat - min_lat)*0.05, min_lon + (max_lon - min_lon)*0.9),
               bbox=dict(facecolor='white', alpha=0.7))
    
    # Add basic stats
    coverage_width = len(parallel_lines) * 3.0 if parallel_lines else 0
    plt.annotate(f"Coverage Width: ~{coverage_width:.1f} m", 
               (min_lat + (max_lat - min_lat)*0.05, min_lon + (max_lon - min_lon)*0.85),
               bbox=dict(facecolor='white', alpha=0.7))
    
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.title('Coverage Analysis with Parallel Lines')
    plt.grid(True)
    plt.savefig('polygon_coverage_analysis.png', dpi=300)
    plt.show()

def get_setback_point(point1, point0, setback_distance=1.0):
    """
    Calculate a point that's setback_distance meters behind point1 toward point0
    
    Args:
        point1: (lat, lon) tuple for waypoint 1
        point0: (lat, lon) tuple for waypoint 0
        setback_distance: Distance in meters to back up
        
    Returns:
        (lat, lon) tuple for the setback point
    """
    # Calculate the bearing from point0 to point1
    bearing_forward = calculate_bearing(point0[0], point0[1], point1[0], point1[1])
    
    # Reverse the bearing (add 180 degrees)
    bearing_backward = (bearing_forward + 180) % 360
    
    # Calculate meter to degree conversions
    meters_per_lat = 111320  # Approximate meters per degree latitude
    meters_per_lon = 111320 * np.cos(np.radians(point1[0]))  # Approximate meters per degree longitude
    
    # Calculate the shift to apply to point1
    lat_shift = setback_distance * np.cos(np.radians(bearing_backward)) / meters_per_lat
    lon_shift = setback_distance * np.sin(np.radians(bearing_backward)) / meters_per_lon
    
    # Calculate the setback point
    setback_lat = point1[0] + lat_shift
    setback_lon = point1[1] + lon_shift
    
    return (setback_lat, setback_lon)

def find_nearest_point(target_point, line):
    """
    Find the point on line that is closest to target_point
    
    Args:
        target_point: (lat, lon) tuple
        line: LineString to find closest point on
        
    Returns:
        (lat, lon) tuple of closest point on line
    """
    # Use Shapely's nearest points functionality
    from shapely.geometry import Point
    from shapely.ops import nearest_points
    
    target = Point(target_point)
    nearest_points_result = nearest_points(target, line)
    nearest = nearest_points_result[1]  # The point on the line
    
    return (nearest.x, nearest.y)

def visualize_complete_path(polygon_data, complete_path, original_lines, start_point=None):
    """Visualize the polygon and complete coverage path"""
    # Extract coordinates
    lats = [point['lat'] for point in polygon_data]
    lons = [point['lon'] for point in polygon_data]
    
    # Create figure
    plt.figure(figsize=(12, 10))
    
    # Plot polygon
    plt.plot(lats, lons, 'ko-', linewidth=2, label='Polygon Boundary')
    plt.fill(lats, lons, alpha=0.1, color='blue')
    
    # Plot first edge in a different color
    plt.plot([lats[0], lats[1]], [lons[0], lons[1]], 'ro-', linewidth=3, label='Reference Edge')
    
    # Plot setback point
    if start_point:
        plt.plot(start_point[0], start_point[1], 'mo', markersize=10, 
                label='Start Point (1m setback)')
    
    # Plot the complete path with color gradient to show direction
    cmap = plt.cm.viridis
    colors = [cmap(i/len(complete_path)) for i in range(len(complete_path))]
    
    # First plot all original lines with thin gray lines to show coverage pattern
    for line in original_lines:
        x, y = line.xy
        plt.plot(x, y, color='gray', linewidth=1, linestyle='--', alpha=0.5)
    
    # Then plot the complete path with color gradient
    for i, line in enumerate(complete_path):
        x, y = line.xy
        
        # Determine line style
        if line in original_lines:
            linestyle = '-'
            linewidth = 2.0
        else:
            linestyle = ':'
            linewidth = 1.5
        
        # Plot the line segment
        plt.plot(x, y, color=colors[i], linewidth=linewidth, linestyle=linestyle)
        
        # Add direction arrows for longer segments
        if len(x) > 5:
            mid_idx = len(x) // 2
            dx = x[mid_idx + 1] - x[mid_idx - 1]
            dy = y[mid_idx + 1] - y[mid_idx - 1]
            plt.arrow(x[mid_idx], y[mid_idx], dx*0.00001, dy*0.00001, 
                     head_width=0.00001, head_length=0.00001, fc=colors[i], ec=colors[i])
    
    # Add start and end markers
    if len(complete_path) > 0:
        first_x, first_y = complete_path[0].xy
        plt.plot(first_x[0], first_y[0], 'go', markersize=10, label='Path Start')
        
        last_x, last_y = complete_path[-1].xy
        plt.plot(last_x[-1], last_y[-1], 'ro', markersize=10, label='Path End')
    
    # Annotate waypoints
    for i, (lat, lon) in enumerate(zip(lats, lons)):
        plt.annotate(f"P{i}", (lat, lon), fontsize=12)
    
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.title('Complete Coverage Path with Connecting Ellipses')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('complete_coverage_path.png', dpi=300)
    plt.show()
    
    # Create a second figure with path statistics
    plt.figure(figsize=(12, 8))
    
    # Calculate statistics
    min_lat, max_lat = min(lats), max(lats)
    min_lon, max_lon = min(lons)
    
    # Calculate approximate total path length
    lat_avg = (min_lat + max_lat) / 2
    meters_per_deg = 111320 * np.cos(np.radians(lat_avg))
    total_length_meters = sum(line.length * meters_per_deg for line in complete_path)
    
    # Create a mini-map of the path
    plt.subplot(1, 2, 1)
    plt.plot(lats, lons, 'k-', linewidth=1)
    
    # Plot the complete path with color gradient
    for i, line in enumerate(complete_path):
        x, y = line.xy
        plt.plot(x, y, color=colors[i], linewidth=1.5)
    
    plt.title('Coverage Path')
    plt.axis('equal')
    plt.grid(True)
    
    # Create a text panel with statistics
    plt.subplot(1, 2, 2)
    plt.axis('off')
    stats_text = (
        f"Coverage Statistics\n"
        f"------------------\n\n"
        f"Parallel Lines: {len(original_lines)}\n"
        f"Line Spacing: 3.0 meters\n"
        f"Connection Paths: {len(complete_path) - len(original_lines)}\n\n"
        f"Total Path Length: {total_length_meters:.1f} meters\n"
        f"Polygon Perimeter: {sum(haversine_distance((lats[i], lons[i]), (lats[(i+1)%len(lats)], lons[(i+1)%len(lats)])) for i in range(len(lats))):.1f} m\n"
    )
    plt.text(0.1, 0.5, stats_text, fontsize=14, va='center')
    
    plt.tight_layout()
    plt.savefig('coverage_path_stats.png', dpi=300)
    plt.show()

def generate_waypoints(complete_path, waypoint_spacing=0.5):
    """
    Generate waypoints along the complete path at the specified spacing
    
    Args:
        complete_path: List of LineString objects representing the complete path
        waypoint_spacing: Distance in meters between waypoints
        
    Returns:
        List of dictionaries with lat/lon keys representing the waypoints
    """
    waypoints = []
    
    for line in complete_path:
        # Get the coordinates of the line
        coords = list(line.coords)
        
        # Calculate the total length of the line
        total_length = line.length * 111320 * np.cos(np.radians(coords[0][0]))
        
        # Calculate the number of segments
        num_segments = int(total_length / waypoint_spacing)
        
        # If the line is too short, skip it
        if num_segments <= 0:
            continue
        
        for i in range(num_segments + 1):
            # Calculate the distance along the line
            distance = i * waypoint_spacing
            
            # Calculate the point at that distance
            point = line.interpolate(distance / 111320 / np.cos(np.radians(coords[0][0])))
            
            # Add the waypoint to the list
            waypoints.append({"lat": point.x, "lon": point.y})
    
    return waypoints


def main():
    # Load polygon data from JSON file
    with open('polygon_data.json') as f:
        polygon_data = json.load(f)
    
    # Generate parallel lines
    original_lines = generate_parallel_lines(polygon_data, spacing=3.0)
    
    # Get waypoints 0 and 1
    waypoint_0 = (polygon_data[0]['lat'], polygon_data[0]['lon'])
    waypoint_1 = (polygon_data[1]['lat'], polygon_data[1]['lon'])
    
    # Calculate a point 1 meter behind waypoint 1 (toward waypoint 0)
    start_point = get_setback_point(waypoint_1, waypoint_0, setback_distance=SETBACK_DISTANCE)
    
    # Create a new list with original lines and connection paths
    complete_path = []
    
    # Add start-to-first elliptical path
    if original_lines and len(original_lines) > 0:
        first_line = original_lines[0]
        first_line_coords = list(first_line.coords)
        
        # Get starting point of the first parallel line - we'll use the leftmost point
        first_line_start = first_line_coords[0]
        
        # Create elliptical path for the beginning
        start_ellipse = create_ellipse_path(start_point, first_line_start, polygon_data, curve_direction=-1)
        
        # Add to complete path
        if start_ellipse:
            # Reverse the order of the coordinates *before* creating the LineString
            reversed_coords = list(start_ellipse.coords)[::-1]
            start_ellipse = LineString(reversed_coords)
            complete_path.append(start_ellipse)
            print(f"Added start elliptical path from setback point to first parallel line")
        else:
            print(f"Start ellipse is invalid (outside polygon)")
    
    # Create connections between all parallel lines in an alternating pattern
    # Track our current direction - starts with first line from left to right
    direction_forward = True
    
    for i in range(len(original_lines)):
        current_line = original_lines[i]
        current_line_coords = list(current_line.coords)
        
        # If we're going backward, reverse the line coordinates
        if not direction_forward:
            # Add reversed version of the line
            current_line = LineString(current_line_coords[::-1])
        
        # Add the current line to the path
        complete_path.append(current_line)
        
        # If there's a next line, add a connection to it
        if i < len(original_lines) - 1:
            next_line = original_lines[i+1]
            next_line_coords = list(next_line.coords)
            
            # Current position is the end of the current line
            current_end = current_line_coords[-1] if direction_forward else current_line_coords[0]
            
            # For the next line, we want the end that's closest to our current position
            dist_start = haversine_distance(current_end, next_line_coords[0])
            dist_end = haversine_distance(current_end, next_line_coords[-1])
            
            # Choose the closest endpoint of the next line
            next_line_point = next_line_coords[0] if dist_start < dist_end else next_line_coords[-1]
            
            # If we chose the end point, next direction will be backward
            next_direction_forward = (next_line_point == next_line_coords[0])
            
            # Create elliptical connection
            curve_direction = 1 if i % 2 == 0 else -1  # Alternate curve direction
            connection_ellipse = create_ellipse_path(current_end, next_line_point, polygon_data, curve_direction=curve_direction)
            
            # Add to complete path
            if connection_ellipse:
                # Reverse the order of the coordinates *before* creating the LineString
                reversed_coords = list(connection_ellipse.coords)[::-1]
                connection_ellipse = LineString(reversed_coords)
                complete_path.append(connection_ellipse)
                print(f"Added connection from line {i+1} to line {i+2}, {'forward' if next_direction_forward else 'backward'}")
            else:
                print(f"Connection ellipse from line {i+1} to line {i+2} is invalid (outside polygon)")
            
            # Update direction for next iteration
            direction_forward = next_direction_forward
    
    # Generate waypoints along the complete path
    waypoints = generate_waypoints(complete_path, waypoint_spacing=0.5)
    
    # Save waypoints to a JSON file
    with open('polygon-to-path.json', 'w') as outfile:
        json.dump(waypoints, outfile, indent=2)
    
    print(f"Generated {len(waypoints)} waypoints and saved to polygon-to-path.json")
    
    # Visualize results with improved visualization
    visualize_complete_path(polygon_data, complete_path, original_lines, start_point)
    
    print(f"Generated zigzag pattern with {len(original_lines)} parallel lines at 3.0 meter spacing")
    print(f"Total path segments: {len(complete_path)}")

if __name__ == "__main__":
    main()
