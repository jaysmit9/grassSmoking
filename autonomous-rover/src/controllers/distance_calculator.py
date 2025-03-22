import numpy as np
from geopy.distance import geodesic

def haversine_distance(coord1, coord2):
    """Calculate the great circle distance between two points in meters"""
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    return geodesic((lat1, lon1), (lat2, lon2)).meters

if __name__ == "__main__":
    # Example usage
    waypoint1 = (34.1519015, -77.8667718)  # Example waypoint 1
    waypoint2 = (34.1519023, -77.8667734)  # Example waypoint 2
    
    distance = haversine_distance(waypoint1, waypoint2)
    print(f"The distance between {waypoint1} and {waypoint2} is: {distance:.2f} meters")