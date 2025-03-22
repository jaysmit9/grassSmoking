import json
import os
import sys
import numpy as np
from geopy.distance import geodesic

# Add the project root to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import the different waypoint loading functions
from utils.map_vis import load_waypoints as map_vis_load
from main import load_waypoints as main_load

def main():
    """Compare waypoint loading between different modules"""
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
    
    # Examine the raw JSON content
    print("\n=== RAW JSON CONTENT ===")
    with open(waypoints_file, 'r') as f:
        json_content = json.load(f)
        print(f"Type: {type(json_content)}")
        print(f"Content structure: {json_content[:5] if isinstance(json_content, list) else json_content.keys()}")
        
        # If it's a list, show the first element
        if isinstance(json_content, list) and len(json_content) > 0:
            print(f"First waypoint data structure: {json_content[0]}")
        
    # Load waypoints using map_vis.py method
    print("\n=== WAYPOINTS LOADED BY MAP_VIS ===")
    try:
        map_vis_waypoints = map_vis_load(waypoints_file)
        print(f"Count: {len(map_vis_waypoints)}")
        print("Sample waypoints:")
        for i, wp in enumerate(map_vis_waypoints[:5]):
            print(f"  WP {i}: {wp}")
    except Exception as e:
        print(f"Error loading with map_vis: {e}")
    
    # Load waypoints using main.py method
    print("\n=== WAYPOINTS LOADED BY MAIN.PY ===")
    try:
        main_waypoints = main_load(waypoints_file)
        print(f"Count: {len(main_waypoints)}")
        print("Sample waypoints:")
        for i, wp in enumerate(main_waypoints[:5]):
            print(f"  WP {i}: {wp}")
    except Exception as e:
        print(f"Error loading with main.py: {e}")
    
    # If both loaded correctly, compare distances
    if 'map_vis_waypoints' in locals() and 'main_waypoints' in locals():
        print("\n=== DISTANCE COMPARISON ===")
        test_position = (34.1519056, -77.8667716)  # Your test position
        
        print(f"Test position: {test_position}")
        print("WP  Map Vis Distance (m)  Main.py Distance (m)  Difference (m)")
        print("-" * 70)
        
        for i in range(min(len(map_vis_waypoints), len(main_waypoints))):
            map_wp = map_vis_waypoints[i]
            main_wp = main_waypoints[i]
            
            map_distance = geodesic(test_position, map_wp).meters
            main_distance = geodesic(test_position, main_wp).meters
            
            diff = map_distance - main_distance
            
            print(f"{i:<3}{map_distance:<21.1f}{main_distance:<21.1f}{diff:<15.1f}")
        
        print("\n=== POTENTIAL COORDINATE SWAPS ===")
        # Check if swapping coordinates would make distances match
        print("WP  Original D(m)  Swapped D(m)  GPS Test D(m)")
        print("-" * 60)
        
        test_distances = [21.8, 26.9, 17.8, 9.2]  # From your map_vis output
        gps_test_distances = [12.0, 18.8, 13.1, 2.1]  # From --gps-test output
        
        for i in range(min(len(map_vis_waypoints), len(main_waypoints))):
            map_wp = map_vis_waypoints[i]
            
            # Try swapping lat/lon
            swapped_wp = (map_wp[1], map_wp[0])
            
            orig_distance = geodesic(test_position, map_wp).meters
            swapped_distance = geodesic(test_position, swapped_wp).meters
            gps_test_d = gps_test_distances[i] if i < len(gps_test_distances) else "N/A"
            
            print(f"{i:<3}{orig_distance:<14.1f}{swapped_distance:<14.1f}{gps_test_d}")

if __name__ == "__main__":
    main()