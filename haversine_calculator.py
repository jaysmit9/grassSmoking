import numpy as np
from geopy.distance import geodesic
import json

def haversine_distance(coord1, coord2):
    """
    Calculate the great-circle distance between two points on earth
    using the haversine formula.
    
    Args:
        coord1: tuple of (latitude, longitude) in decimal degrees
        coord2: tuple of (latitude, longitude) in decimal degrees
        1
    Returns:
        Distance in meters between the two coordinates
    """
    return geodesic(coord1, coord2).meters

def manual_haversine(coord1, coord2):
    """
    Calculate haversine distance without using geodesic
    
    Args:
        coord1: tuple of (latitude, longitude) in decimal degrees
        coord2: tuple of (latitude, longitude) in decimal degrees
        
    Returns:
        Distance in meters between the two coordinates
    """
    # Earth radius in meters
    R = 6371000
    
    # Convert decimal degrees to radians
    lat1, lon1 = np.radians(coord1[0]), np.radians(coord1[1])
    lat2, lon2 = np.radians(coord2[0]), np.radians(coord2[1])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    # Distance in meters
    distance = R * c
    
    return distance

def print_waypoint_distances(waypoints):
    """Print distances between consecutive waypoints"""
    print("\n===== Distances Between Consecutive Waypoints =====")
    total_distance = 0
    
    for i in range(len(waypoints)-1):
        coord1 = (waypoints[i][0], waypoints[i][1])
        coord2 = (waypoints[i+1][0], waypoints[i+1][1])
        distance = haversine_distance(coord1, coord2)
        total_distance += distance
        
        print(f"Waypoint {i} to {i+1}: {distance:.2f} meters")
        print(f"  Waypoint {i}: ({coord1[0]:.6f}, {coord1[1]:.6f})")
        print(f"  Waypoint {i+1}: ({coord2[0]:.6f}, {coord2[1]:.6f})")
    
    print(f"\nTotal path length: {total_distance:.2f} meters")

def main():
    print("==== Haversine Distance Calculator ====")
    print("This program calculates the distance between GPS coordinates using the Haversine formula")
    
    # Try to load waypoints from polygon_data.json
    try:
        with open('polygon_data.json') as f:
            polygon_data = json.load(f)
            waypoints = [(point['lat'], point['lon']) for point in polygon_data]
            print(f"Loaded {len(waypoints)} waypoints from polygon_data.json")
            print_waypoint_distances(waypoints)
    except (FileNotFoundError, json.JSONDecodeError) as e:
        print(f"Could not load polygon_data.json: {e}")
        waypoints = []
    
    # Interactive mode
    print("\n===== Interactive Calculator =====")
    print("Enter coordinates to calculate distances (or 'q' to quit)")
    
    # Get coordinates from debug_output.txt
    iteration_150 = (34.15217954177397, -77.86682260447695)
    waypoint_3 = None  # We don't know the exact coordinates of waypoint 3 yet
    
    while True:
        print("\nOptions:")
        print("1. Calculate distance between two points")
        print("2. Calculate distance to iteration 150 position")
        print("3. Calculate distance between specific waypoints")
        print("q. Quit")
        
        choice = input("Enter choice: ")
        
        if choice.lower() == 'q':
            break
        
        if choice == '1':
            try:
                lat1 = float(input("Enter latitude 1: "))
                lon1 = float(input("Enter longitude 1: "))
                lat2 = float(input("Enter latitude 2: "))
                lon2 = float(input("Enter longitude 2: "))
                
                coord1 = (lat1, lon1)
                coord2 = (lat2, lon2)
                
                distance = haversine_distance(coord1, coord2)
                manual_dist = manual_haversine(coord1, coord2)
                
                print(f"Distance: {distance:.2f} meters (geopy)")
                print(f"Distance: {manual_dist:.2f} meters (manual calculation)")
                
            except ValueError:
                print("Invalid input. Please enter valid coordinates.")
        
        elif choice == '2':
            try:
                lat = float(input("Enter latitude: "))
                lon = float(input("Enter longitude: "))
                
                coord = (lat, lon)
                
                distance = haversine_distance(coord, iteration_150)
                print(f"Distance to iteration 150 position: {distance:.2f} meters")
                
            except ValueError:
                print("Invalid input. Please enter valid coordinates.")
        
        elif choice == '3':
            if not waypoints:
                print("No waypoints available.")
                continue
                
            try:
                print("Available waypoints:")
                for i, wp in enumerate(waypoints):
                    print(f"{i}: ({wp[0]:.6f}, {wp[1]:.6f})")
                
                idx1 = int(input("Enter first waypoint index: "))
                idx2 = int(input("Enter second waypoint index: "))
                
                if 0 <= idx1 < len(waypoints) and 0 <= idx2 < len(waypoints):
                    distance = haversine_distance(waypoints[idx1], waypoints[idx2])
                    print(f"Distance from waypoint {idx1} to {idx2}: {distance:.2f} meters")
                else:
                    print("Invalid waypoint indices.")
            
            except ValueError:
                print("Invalid input. Please enter valid indices.")

if __name__ == "__main__":
    main()