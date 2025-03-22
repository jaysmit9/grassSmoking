#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/static_map.py

import folium
import argparse
import json
import os
import sys
import time
import math
import logging
from geopy.distance import geodesic

# Add these imports at the top, near the other imports
import sys
import os

# Add the project root to the Python path to allow importing from hardware
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import the GPS monitor class
try:
    from hardware.gps_monitor import GPSMonitor
except ImportError as e:
    logger.warning(f"Could not import GPSMonitor: {e}")
    logger.warning("Live GPS position will not be available")
    GPSMonitor = None

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate bearing between two points using the Great Circle formula.
    Always returns bearing in degrees (0-360).
    """
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    
    # Calculate bearing
    dlon = lon2_rad - lon1_rad
    y = math.sin(dlon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    bearing = math.atan2(y, x)
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = math.degrees(bearing)
    bearing_deg = (bearing_deg + 360) % 360
    
    return bearing_deg

def calculate_distance(lat1, lon1, lat2, lon2, method='geodesic'):
    """
    Calculate distance between two points using various methods.
    
    Args:
        lat1, lon1: Starting point coordinates
        lat2, lon2: Ending point coordinates
        method: 'geodesic' (most accurate), 'haversine' (good approximation), 
                'simple' (flat earth approximation), or 'pythagoras' (simple 2D)
    
    Returns:
        Distance in meters
    """
    if method == 'geodesic':
        # Most accurate for Earth's ellipsoid shape
        from geopy.distance import geodesic
        return geodesic((lat1, lon1), (lat2, lon2)).meters
    
    elif method == 'haversine':
        # Great-circle distance - assumes spherical earth
        import math
        R = 6371000  # Earth's radius in meters
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula
        dLat = lat2_rad - lat1_rad
        dLon = lon2_rad - lon1_rad
        a = math.sin(dLat/2) * math.sin(dLat/2) + \
            math.cos(lat1_rad) * math.cos(lat2_rad) * \
            math.sin(dLon/2) * math.sin(dLon/2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        return distance
    
    elif method == 'simple':
        # Simplified calculation for very short distances
        # Uses the equirectangular approximation which is faster but less accurate
        import math
        R = 6371000  # Earth's radius in meters
        
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Simple formula
        x = (lon2_rad - lon1_rad) * math.cos((lat1_rad + lat2_rad) / 2)
        y = lat2_rad - lat1_rad
        distance = math.sqrt(x*x + y*y) * R
        return distance
        
    elif method == 'pythagoras':
        # Ultra simple 2D calculation - just for comparison with GPS test
        # Convert degrees to approximate meters at this latitude
        # (very rough approximation - only useful for very short distances)
        import math
        lat_to_m = 111111  # 1 degree latitude ~= 111,111 meters
        lon_to_m = 111111 * math.cos(math.radians(lat1))  # 1 degree longitude depends on latitude
        
        # Simple Pythagorean theorem
        dx = (lon2 - lon1) * lon_to_m  # longitude difference in meters
        dy = (lat2 - lat1) * lat_to_m  # latitude difference in meters
        distance = math.sqrt(dx*dx + dy*dy)
        return distance
    
    else:
        raise ValueError(f"Unknown distance calculation method: {method}")

def load_waypoints(waypoints_file):
    """Load waypoints from file."""
    try:
        with open(waypoints_file, 'r') as f:
            if waypoints_file.endswith('.json'):
                data = json.load(f)
                if isinstance(data, list):
                    if len(data) > 0 and all('lat' in p and 'lon' in p for p in data):
                        # Format where each point is {lat: X, lon: Y}
                        waypoints = [(p['lat'], p['lon']) for p in data]
                        logger.info("Loaded waypoints from JSON with lat/lon keys")
                    else:
                        # Try other JSON formats
                        if isinstance(data, dict) and 'waypoints' in data:
                            waypoints = [(p['lat'], p['lon']) for p in data['waypoints']]
                            logger.info("Loaded waypoints from JSON waypoints array")
                        else:
                            # Try to extract as array of coordinates
                            try:
                                # Format is [[lat, lon], ...]
                                waypoints = [(p[0], p[1]) for p in data]
                                logger.info("Loaded waypoints from JSON array")
                            except:
                                waypoints = []
                                logger.error("Could not determine JSON format")
            else:
                # Assume simple lat,lon format
                waypoints = []
                for line in f.readlines():
                    try:
                        parts = line.strip().split(',')
                        if len(parts) >= 2:
                            lat = float(parts[0])
                            lon = float(parts[1])
                            waypoints.append((lat, lon))
                    except ValueError:
                        continue
        
        logger.info(f"Loaded {len(waypoints)} waypoints")
        return waypoints
        
    except Exception as e:
        logger.error(f"Error loading waypoints: {e}")
        return []

def create_static_map(waypoints, current_position=None, current_heading=None, output_file="static_rover_map.html", distance_method='geodesic'):
    """Create a static map with waypoints and information."""
    if not waypoints:
        logger.error("No waypoints to visualize")
        return
    
    # Use first waypoint as current position if not provided
    if current_position is None:
        current_position = waypoints[0]
    
    current_lat, current_lon = current_position
    
    # Calculate map center
    center_lat = (current_lat + sum(wp[0] for wp in waypoints)) / (len(waypoints) + 1)
    center_lon = (current_lon + sum(wp[1] for wp in waypoints)) / (len(waypoints) + 1)
    
    # Create map
    m = folium.Map(location=[center_lat, center_lon], zoom_start=19)
    
    # Add current position marker
    folium.Marker(
        [current_lat, current_lon],
        popup="Current Position",
        icon=folium.Icon(color="red", icon="crosshairs", prefix='fa'),
        tooltip="Current Position"
    ).add_to(m)
    
    # Add waypoints with info
    waypoint_table = "<table style='width:100%; border-collapse: collapse;'>"
    waypoint_table += "<tr style='background-color:#f2f2f2'><th>WP</th><th>Distance (m)</th><th>Bearing</th><th>Turn</th></tr>"
    
    for i, (wp_lat, wp_lon) in enumerate(waypoints):
        # Calculate distance using the specified method
        distance = calculate_distance(current_lat, current_lon, wp_lat, wp_lon, method=distance_method)
        
        # Calculate bearing
        bearing = calculate_bearing(current_lat, current_lon, wp_lat, wp_lon)
        
        # Calculate turn direction if heading is provided
        turn_info = ""
        if current_heading is not None:
            relative_bearing = bearing - current_heading
            relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
            
            if abs(relative_bearing) < 1:
                turn_direction = "STRAIGHT"
                turn_info = "STRAIGHT"
            elif relative_bearing > 0:
                turn_direction = "RIGHT"
                turn_info = f"→ {abs(relative_bearing):.1f}°"
            else:
                turn_direction = "LEFT" 
                turn_info = f"← {abs(relative_bearing):.1f}°"
        
        # Add to table
        row_style = "style='border-bottom:1px solid #ddd;'" if i < len(waypoints) - 1 else ""
        waypoint_table += f"<tr {row_style}>"
        waypoint_table += f"<td>{i}</td><td>{distance:.1f}</td><td>{bearing:.1f}°</td><td>{turn_info}</td>"
        waypoint_table += "</tr>"
        
        # Add marker with popup containing distance and bearing
        popup_html = f"""
        <div style="font-family: Arial; min-width: 150px;">
            <b>Waypoint {i}</b><br>
            Distance: {distance:.1f} m<br>
            Bearing: {bearing:.1f}°<br>
            {turn_info}
        </div>
        """
        
        folium.Marker(
            [wp_lat, wp_lon],
            popup=folium.Popup(popup_html, max_width=200),
            icon=folium.Icon(color="blue", icon="flag", prefix='fa'),
            tooltip=f"Waypoint {i}"
        ).add_to(m)
    
    waypoint_table += "</table>"
    
    # Connect waypoints with a line
    folium.PolyLine(
        waypoints,
        color="blue",
        weight=2.5,
        opacity=0.8,
    ).add_to(m)
    
    # Add heading line if provided
    if current_heading is not None:
        # Convert heading to radians and calculate end point
        heading_rad = math.radians(current_heading)
        length = 0.00020  # Longer for visibility
        
        # Correct calculation for heading line endpoint
        # Here longitude corresponds to x-coordinate (east-west) 
        # and latitude to y-coordinate (north-south)
        end_lat = current_lat + length * math.cos(heading_rad)
        end_lon = current_lon + length * math.sin(heading_rad)
        
        # Add the heading line
        folium.PolyLine(
            [(current_lat, current_lon), (end_lat, end_lon)],
            color="red",
            weight=3,
            opacity=1.0,
            tooltip=f"Heading: {current_heading:.1f}°"
        ).add_to(m)
    
    # Add info panel
    info_html = f"""
    <div style="
        position: absolute; 
        top: 10px;
        right: 10px;
        z-index: 1000;
        background-color: white;
        padding: 10px;
        border-radius: 5px;
        box-shadow: 0 0 10px rgba(0,0,0,0.5);
        font-family: Arial;
        max-width: 300px;
        max-height: 80%;
        overflow-y: auto;
    ">
        <h3 style="margin-top: 0;">Waypoint Information</h3>
        <div>Current Position: ({current_lat:.7f}, {current_lon:.7f})</div>
        {f'<div>Current Heading: {current_heading:.1f}°</div>' if current_heading is not None else ''}
        <div style="margin-top: 10px;">
            {waypoint_table}
        </div>
    </div>
    """
    
    # Add the info panel to the map
    m.get_root().html.add_child(folium.Element(info_html))
    
    # Save the map
    m.save(output_file)
    logger.info(f"Map created at {output_file}")
    
    # Print the same information to the console
    print("\n=== WAYPOINT INFORMATION ===")
    print(f"Current Position: ({current_lat:.7f}, {current_lon:.7f})")
    if current_heading is not None:
        print(f"Current Heading: {current_heading:.1f}°")
    print(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
    print("-" * 40)
    
    for i, (wp_lat, wp_lon) in enumerate(waypoints):
        distance = calculate_distance(current_lat, current_lon, wp_lat, wp_lon, method=distance_method)
        bearing = calculate_bearing(current_lat, current_lon, wp_lat, wp_lon)
        
        turn = ""
        if current_heading is not None:
            relative_bearing = bearing - current_heading
            relative_bearing = ((relative_bearing + 180) % 360) - 180
            
            if abs(relative_bearing) < 1:
                turn = "STRAIGHT"
            elif relative_bearing > 0:
                turn = f"→ {abs(relative_bearing):.1f}°"
            else:
                turn = f"← {abs(relative_bearing):.1f}°"
        
        print(f"{i:<4}{distance:<14.1f}{bearing:<10.1f}°{turn}")
    
    print("=" * 40)
    return m

def create_auto_refreshing_map(waypoints, current_position, current_heading=None, 
                             output_file="live_rover_map.html", distance_method='geodesic',
                             refresh_interval=5):
    """Create a map that auto-refreshes to show the current position."""
    # First create a standard map
    m = create_static_map(waypoints, current_position, current_heading, output_file, distance_method)
    
    # Add auto-refresh meta tag to the HTML
    with open(output_file, 'r') as f:
        html_content = f.read()
    
    # Add meta refresh tag in the head section
    if '<head>' in html_content:
        refreshed_content = html_content.replace('<head>', 
                                               f'<head>\n<meta http-equiv="refresh" content="{refresh_interval}">')
        
        # Add a note about auto-refreshing
        refreshed_content = refreshed_content.replace('</body>', 
                                                   f'<div style="position:fixed; bottom:10px; left:10px; background:rgba(255,255,255,0.7); padding:5px; border-radius:5px;">'
                                                   f'Auto-refreshing every {refresh_interval} seconds</div>\n</body>')
        
        # Write the modified content back
        with open(output_file, 'w') as f:
            f.write(refreshed_content)
            
        logger.info(f"Created auto-refreshing map at {output_file} (refreshing every {refresh_interval} seconds)")
    
    return m

def add_gps_comparison(parser):
    """Add GPS comparison option to the argument parser"""
    parser.add_argument('--gps-lat', type=float, help='Current GPS latitude for comparison')
    parser.add_argument('--gps-lon', type=float, help='Current GPS longitude for comparison')
    parser.add_argument('--target-idx', type=int, default=0, help='Target waypoint index for comparison')
    
    return parser

# Update the main function to add a live GPS option
def main():
    parser = argparse.ArgumentParser(description='Static Map Visualizer for Rover')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file (default: data/polygon_data.json)', default=None)
    parser.add_argument('--position', '-p', help='Current position as lat,lon', default=None)
    parser.add_argument('--heading', '-H', type=float, help='Current heading in degrees', default=None)
    parser.add_argument('--output', '-o', help='Output HTML file path', default="static_rover_map.html")
    parser.add_argument('--method', '-m', choices=['geodesic', 'haversine', 'simple', 'pythagoras'], 
                      default='geodesic', help='Distance calculation method')
    parser.add_argument('--compare', '-c', action='store_true', help='Compare all distance calculation methods')
    parser.add_argument('--live', '-l', action='store_true', help='Use live GPS position from rover')
    parser.add_argument('--refresh', '-r', type=int, default=5, 
                       help='Auto-refresh interval in seconds (with --live)')
    
    # Add GPS comparison options
    parser = add_gps_comparison(parser)
    
    args = parser.parse_args()
    
    # If waypoints file not specified, try to use the default polygon_data.json
    if args.waypoints is None:
        # Try different possible locations for the default file
        possible_paths = [
            'data/polygon_data.json',
            '../data/polygon_data.json',
            '../../data/polygon_data.json',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/polygon_data.json'),
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/polygon_data.json')
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                args.waypoints = path
                logger.info(f"Using default waypoints file: {path}")
                break
        
        if args.waypoints is None:
            logger.error("Default waypoints file not found. Please specify a waypoints file.")
            sys.exit(1)
    
    # Load waypoints
    waypoints = load_waypoints(args.waypoints)
    
    if not waypoints:
        logger.error("No waypoints loaded. Exiting.")
        sys.exit(1)
    
    # Now handle GPS comparison AFTER loading waypoints
    if args.gps_lat is not None and args.gps_lon is not None:
        gps_position = (args.gps_lat, args.gps_lon)
        
        if args.target_idx < len(waypoints):
            target = waypoints[args.target_idx]
            
            print("\n===== DIRECT GPS COMPARISON =====")
            print(f"GPS Position: ({gps_position[0]:.7f}, {gps_position[1]:.7f})")
            print(f"Target WP {args.target_idx}: ({target[0]:.7f}, {target[1]:.7f})")
            
            # Calculate distance using all methods
            geodesic_dist = calculate_distance(gps_position[0], gps_position[1], target[0], target[1], 'geodesic')
            haversine_dist = calculate_distance(gps_position[0], gps_position[1], target[0], target[1], 'haversine')
            simple_dist = calculate_distance(gps_position[0], gps_position[1], target[0], target[1], 'simple')  
            pythagoras_dist = calculate_distance(gps_position[0], gps_position[1], target[0], target[1], 'pythagoras')
            
            # Calculate bearing
            bearing = calculate_bearing(gps_position[0], gps_position[1], target[0], target[1])
            
            print(f"Geodesic Distance: {geodesic_dist:.1f} meters")
            print(f"Haversine Distance: {haversine_dist:.1f} meters")
            print(f"Simple Distance: {simple_dist:.1f} meters")
            print(f"Pythagoras Distance: {pythagoras_dist:.1f} meters")
            print(f"Bearing: {bearing:.1f}°")
            print("================================")
            
            # Continue with map visualization
            args.position = f"{gps_position[0]},{gps_position[1]}"
    
    # Get live GPS position if requested
    if args.live:
        # Check if GPSMonitor is available
        if GPSMonitor is None:
            logger.error("GPSMonitor not available. Cannot use live GPS position.")
            sys.exit(1)
            
        try:
            logger.info("Initializing GPS monitor to get live position...")
            # First try with REAL hardware GPS
            gps = GPSMonitor(simulation_mode=False)
            logger.info("Using real GPS hardware")
            
            # Wait for a valid GPS fix
            max_attempts = 15  # First try hardware for 15 seconds
            attempt = 0
            got_fix = False
            
            while attempt < max_attempts and not got_fix:
                try:
                    lat, lon, heading, speed = gps.get_position_and_heading()
                    logger.info(f"GPS data: lat={lat}, lon={lon}, heading={heading}, speed={speed}")
                    
                    if lat is not None and lon is not None:
                        logger.info(f"✅ Got valid GPS position: ({lat:.7f}, {lon:.7f})")
                        current_position = (lat, lon)
                        got_fix = True
                        
                        # If no heading was provided but GPS has heading, use it
                        if args.heading is None and heading is not None:
                            args.heading = heading
                            logger.info(f"Using GPS heading: {heading:.1f}°")
                        break
                except Exception as e:
                    logger.error(f"Error getting position: {e}")
                
                logger.info(f"Waiting for GPS fix... ({attempt+1}/{max_attempts})")
                attempt += 1
                time.sleep(1)
            
            # If hardware GPS didn't work, use a reasonable test position near your waypoints
            if not got_fix:
                logger.warning("Could not get real GPS position. Using nearby test position.")
                
                # Clean up first GPS instance
                if gps:
                    gps.stop()
                
                # Try to find a reasonable position near your first waypoint
                if waypoints:
                    # First waypoint position
                    wp0_lat, wp0_lon = waypoints[0]
                    
                    # Use a position 15 meters from the first waypoint
                    # This is more reasonable than the default simulator position
                    import math
                    # Move slightly west and south of the first waypoint
                    # Approximately 10m south and 10m west
                    offset_lat = -0.00009  # ~10m south
                    offset_lon = -0.00012  # ~10m west at this latitude
                    
                    test_lat = wp0_lat + offset_lat
                    test_lon = wp0_lon + offset_lon
                    
                    logger.info(f"Using test position near first waypoint: ({test_lat:.7f}, {test_lon:.7f})")
                    current_position = (test_lat, test_lon)
                    got_fix = True
                else:
                    # Use the default test position
                    current_position = (34.1519056, -77.8667716)
                    logger.info(f"Using default test position: {current_position}")
                    got_fix = True
            
            if got_fix:
                logger.info(f"Creating map with position: {current_position}")
                
                # Create the map
                if args.refresh > 0:
                    # Create auto-refreshing map
                    create_auto_refreshing_map(waypoints, current_position, args.heading, 
                                              args.output, args.method, args.refresh)
                else:
                    # Create standard map
                    create_static_map(waypoints, current_position, args.heading, 
                                      args.output, args.method)
                
                # Open the map in a web browser
                import webbrowser
                webbrowser.open('file://' + os.path.abspath(args.output))
                return
            else:
                logger.error("Could not get a valid position for mapping.")
                sys.exit(1)
        except Exception as e:
            logger.error(f"Error getting live GPS position: {e}")
            import traceback
            logger.error(traceback.format_exc())
            logger.error("Using fallback position.")
    
    # Parse current position if provided via command line
    current_position = None
    if args.position:
        try:
            lat, lon = map(float, args.position.split(','))
            current_position = (lat, lon)
        except:
            logger.warning(f"Could not parse position '{args.position}'. Using first waypoint.")
    
    # If no position provided or couldn't get GPS, use hardcoded test coordinates
    if current_position is None:
        # Use our test coordinates, which should show the correct 282° bearing
        current_position = (34.1519056, -77.8667716)
        logger.info(f"Using hardcoded position for testing: {current_position}")
    
    # Compare distance methods if requested
    if args.compare:
        print("\n=== DISTANCE CALCULATION COMPARISON ===")
        print(f"Current Position: ({current_position[0]:.7f}, {current_position[1]:.7f})")
        print(f"{'WP':<4}{'Geodesic (m)':<14}{'Haversine (m)':<14}{'Simple (m)':<14}{'Pythagoras (m)':<14}")
        print("-" * 70)
        
        for i, (wp_lat, wp_lon) in enumerate(waypoints):
            d_geodesic = calculate_distance(current_position[0], current_position[1], wp_lat, wp_lon, 'geodesic')
            d_haversine = calculate_distance(current_position[0], current_position[1], wp_lat, wp_lon, 'haversine')
            d_simple = calculate_distance(current_position[0], current_position[1], wp_lat, wp_lon, 'simple')
            d_pythagoras = calculate_distance(current_position[0], current_position[1], wp_lat, wp_lon, 'pythagoras')
            
            print(f"{i:<4}{d_geodesic:<14.1f}{d_haversine:<14.1f}{d_simple:<14.1f}{d_pythagoras:<14.1f}")
        
        print("=" * 70)
    
    # Update create_static_map to use the specified method
    create_static_map(waypoints, current_position, args.heading, args.output, args.method if hasattr(args, 'method') else 'geodesic')
    
    # Open the map in a web browser
    import webbrowser
    webbrowser.open('file://' + os.path.abspath(args.output))

if __name__ == "__main__":
    main()