import os
import sys
import math
from geopy.distance import geodesic

# Add the project root to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import necessary modules
try:
    from hardware.gps_monitor import GPSMonitor
    from main import load_waypoints
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Make sure you're running this from the correct directory.")
    sys.exit(1)

def analyze_distance_calculation(position, waypoints):
    """Analyze the distance calculation methods compared to GPS test values."""
    print("\n===== DISTANCE CALCULATION ANALYSIS =====")
    print(f"Current position: {position}")
    
    # Known distances from GPS test output
    gps_test_distances = [12.0, 18.8, 13.1, 2.1]
    
    # Check different calculation methods
    methods = {
        "Geodesic": lambda p1, p2: geodesic(p1, p2).meters,
        "Direct Line (2D)": lambda p1, p2: calculate_direct_distance(p1, p2),
        "50% of Geodesic": lambda p1, p2: geodesic(p1, p2).meters * 0.5,
        "55% of Geodesic": lambda p1, p2: geodesic(p1, p2).meters * 0.55,
        "60% of Geodesic": lambda p1, p2: geodesic(p1, p2).meters * 0.6,
        "Flat Earth (m)": lambda p1, p2: flat_earth_calculation(p1, p2),
    }
    
    # Print header row
    print(f"{'WP':<3}{'GPS Test':<10}", end="")
    for method_name in methods.keys():
        print(f"{method_name:<15}", end="")
    print("Best Match")
    print("-" * 80)
    
    for i, waypoint in enumerate(waypoints[:len(gps_test_distances)]):
        # Get GPS test distance
        gps_test = gps_test_distances[i]
        
        # Calculate distances using different methods
        calculated_distances = {}
        for method_name, method_func in methods.items():
            dist = method_func(position, waypoint)
            calculated_distances[method_name] = dist
        
        # Find the method with the closest result to GPS test
        best_match = min(calculated_distances.items(), key=lambda x: abs(x[1] - gps_test))
        
        # Print row
        print(f"{i:<3}{gps_test:<10.1f}", end="")
        for method_name in methods.keys():
            print(f"{calculated_distances[method_name]:<15.1f}", end="")
        print(f"{best_match[0]} ({abs(best_match[1] - gps_test):.1f}m diff)")
    
    print("\n=== DETAILED WAYPOINT ANALYSIS ===")
    for i, waypoint in enumerate(waypoints[:len(gps_test_distances)]):
        print(f"\nWaypoint {i}:")
        print(f"  Coordinates: {waypoint}")
        print(f"  GPS Test distance: {gps_test_distances[i]} meters")
        
        # Analyze the distance discrepancy
        geodesic_dist = geodesic(position, waypoint).meters
        ratio = gps_test_distances[i] / geodesic_dist
        print(f"  Geodesic distance: {geodesic_dist:.2f} meters")
        print(f"  Ratio (GPS Test / Geodesic): {ratio:.2f}")
        
        # Check if this matches a simple percentage
        closest_percent = round(ratio * 100)
        print(f"  Appears to be approximately {closest_percent}% of the geodesic distance")
    
    print("\n=== CONCLUSION ===")
    # Check if all ratios are similar
    ratios = [gps_test_distances[i] / geodesic(position, waypoint).meters 
              for i, waypoint in enumerate(waypoints[:len(gps_test_distances)])]
    avg_ratio = sum(ratios) / len(ratios)
    ratio_variance = sum((r - avg_ratio)**2 for r in ratios) / len(ratios)
    
    if ratio_variance < 0.01:  # Low variance indicates consistent scaling
        print(f"The GPS Test distances appear to be consistently scaled at {avg_ratio:.2f} (about {round(avg_ratio*100)}%)")
        print(f"of the actual geodesic distances. This suggests a deliberate scaling factor.")
    else:
        print("The GPS Test distances do not follow a consistent scaling pattern.")
        print("They might be using a different calculation method or coordinate system.")

def calculate_direct_distance(p1, p2):
    """Calculate 'as the crow flies' distance for GPS coordinates."""
    import math
    # Earth's radius in meters
    R = 6371000
    
    # Convert to radians
    lat1 = math.radians(p1[0])
    lon1 = math.radians(p1[1])
    lat2 = math.radians(p2[0])
    lon2 = math.radians(p2[1])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = R * c
    
    return distance

def flat_earth_calculation(p1, p2):
    """Calculate distance using a flat earth approximation."""
    import math
    # Convert degrees to approximate meters at this latitude
    lat_to_m = 111111  # 1 degree latitude ~= 111,111 meters
    lon_to_m = 111111 * math.cos(math.radians(p1[0]))  # 1 degree longitude depends on latitude
    
    # Simple Pythagorean theorem
    dx = (p2[1] - p1[1]) * lon_to_m  # longitude difference in meters
    dy = (p2[0] - p1[0]) * lat_to_m  # latitude difference in meters
    distance = math.sqrt(dx*dx + dy*dy)
    
    return distance

def main():
    # Find the polygon data file
    possible_paths = [
        'data/polygon_data.json',
        '../data/polygon_data.json',
        '../../data/polygon_data.json',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/polygon_data.json'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/polygon_data.json')
    ]
    
    waypoints_file = None
    for path in possible_paths:
        if os.path.exists(path):
            waypoints_file = path
            print(f"Found polygon data file: {path}")
            break
    
    if waypoints_file is None:
        print("Could not find the polygon data file!")
        return
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Use the test position
    position = (34.1519056, -77.8667716)
    print(f"Using test position: {position}")
    
    # Analyze distance calculations
    analyze_distance_calculation(position, waypoints)

if __name__ == "__main__":
    main()