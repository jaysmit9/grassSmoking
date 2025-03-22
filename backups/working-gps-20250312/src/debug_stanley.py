#!/usr/bin/env python3
"""
Stanley Controller Interactive Debugger
This script allows step-by-step execution of the bearing calculations
"""
import numpy as np
import json
import math
import time
import logging
import sys
from geopy.distance import geodesic
from hardware.gps_monitor import GPSMonitor

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate bearing between two points using the map_visualizer formula.
    """
    print(f"\n=== BEARING CALCULATION STEPS ===")
    print(f"Input: Start=({lat1:.7f}, {lon1:.7f}), End=({lat2:.7f}, {lon2:.7f})")
    
    # Convert to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)
    print(f"Step 1: Convert to radians")
    print(f"  lat1_rad = {lat1_rad:.6f}, lon1_rad = {lon1_rad:.6f}")
    print(f"  lat2_rad = {lat2_rad:.6f}, lon2_rad = {lon2_rad:.6f}")
    input("Press Enter to continue...")
    
    # Calculate bearing components
    dlon = lon2_rad - lon1_rad
    print(f"Step 2: Calculate dlon = {dlon:.6f} radians ({math.degrees(dlon):.2f}°)")
    input("Press Enter to continue...")
    
    y = math.sin(dlon) * math.cos(lat2_rad)
    print(f"Step 3: Calculate y = sin(dlon) * cos(lat2_rad)")
    print(f"  sin({dlon:.6f}) = {math.sin(dlon):.6f}")
    print(f"  cos({lat2_rad:.6f}) = {math.cos(lat2_rad):.6f}")
    print(f"  y = {y:.6f}")
    input("Press Enter to continue...")
    
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
    print(f"Step 4: Calculate x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)")
    print(f"  cos({lat1_rad:.6f}) = {math.cos(lat1_rad):.6f}")
    print(f"  sin({lat2_rad:.6f}) = {math.sin(lat2_rad):.6f}")
    print(f"  sin({lat1_rad:.6f}) = {math.sin(lat1_rad):.6f}")
    print(f"  cos({lat2_rad:.6f}) = {math.cos(lat2_rad):.6f}")
    print(f"  cos({dlon:.6f}) = {math.cos(dlon):.6f}")
    print(f"  x = {x:.6f}")
    input("Press Enter to continue...")
    
    bearing_rad = math.atan2(y, x)
    print(f"Step 5: Calculate bearing_rad = atan2(y, x)")
    print(f"  atan2({y:.6f}, {x:.6f}) = {bearing_rad:.6f} radians")
    input("Press Enter to continue...")
    
    # Convert to degrees and normalize to 0-360
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360
    print(f"Step 6: Convert to degrees and normalize to 0-360")
    print(f"  {bearing_rad:.6f} radians = {bearing_deg:.2f}°")
    input("Press Enter to continue...")
    
    print(f"=== RESULT: Bearing = {bearing_deg:.2f}° ===\n")
    return bearing_deg

def debug_stanley_control():
    """
    Interactive debugger for the Stanley Controller's bearing calculation
    """
    try:
        # Get GPS reading first
        gps = GPSMonitor()
        print("Getting current GPS position...")
        lat, lon, heading, speed = gps.get_position_and_heading()
        
        if lat is None or lon is None:
            print("Error: GPS position unavailable. Please ensure GPS is connected.")
            return
            
        print(f"Current Position: ({lat:.7f}, {lon:.7f})")
        print(f"Current Heading: {heading:.1f}°")
        
        # Ask for waypoint data
        print("\nPlease enter target waypoint coordinates:")
        mode = input("Choose mode - (1) Enter coordinates manually, (2) Use waypoints file: ")
        
        target_lat, target_lon = None, None
        
        if mode == "1":
            try:
                target_lat = float(input("Target latitude: "))
                target_lon = float(input("Target longitude: "))
                waypoint_idx = 0
            except ValueError:
                print("Error: Invalid coordinates. Please enter numeric values.")
                return
        else:
            # Get waypoints from file
            waypoints_file = input("Enter waypoints file path (default: ../../data/polygon_data.json): ") or "../data/polygon_data.json"
            try:
                with open(waypoints_file, 'r') as f:
                    data = json.load(f)
                
                if isinstance(data, list) and len(data) > 0:
                    # Extract waypoints based on format
                    if isinstance(data[0], list):  # Simple format [[lat, lon], ...]
                        waypoints = np.array(data, dtype=np.float64)
                    elif isinstance(data[0], dict) and 'lat' in data[0] and 'lon' in data[0]:
                        waypoints = np.array([[point['lat'], point['lon']] for point in data], dtype=np.float64)
                    else:
                        print("Error: Unknown waypoint format in file.")
                        return
                        
                    # Print all waypoints
                    print("\nAvailable waypoints:")
                    for i, wp in enumerate(waypoints):
                        print(f"[{i}]: ({wp[0]:.7f}, {wp[1]:.7f})")
                        
                    # Let user choose which waypoint to use
                    waypoint_idx = int(input(f"Enter waypoint index (0-{len(waypoints)-1}): "))
                    if waypoint_idx < 0 or waypoint_idx >= len(waypoints):
                        print("Error: Invalid waypoint index.")
                        return
                        
                    target_lat, target_lon = waypoints[waypoint_idx]
                else:
                    print("Error: No waypoints found in file.")
                    return
            except Exception as e:
                print(f"Error loading waypoints: {e}")
                return
        
        print(f"\nSelected Target: ({target_lat:.7f}, {target_lon:.7f})")
        input("Press Enter to begin step-by-step debugging...")
        
        # First test the visualization formula
        print("\n==== MAP VISUALIZER BEARING CALCULATION ====")
        bearing = calculate_bearing(lat, lon, target_lat, target_lon)
        
        # Calculate heading error
        heading_error = bearing - heading
        heading_error = ((heading_error + 180) % 360) - 180  # Add closing parenthesis
        
        print(f"Current GPS Heading: {heading:.1f}°")
        print(f"Calculated Bearing: {bearing:.1f}°")
        print(f"Heading Error: {heading_error:.1f}° {'RIGHT' if heading_error > 0 else 'LEFT'}")
        
        # Now test the Stanley Controller formula with various parameter orderings
        print("\n==== TESTING DIFFERENT PARAMETER ORDERINGS ====")
        
        # Original ordering from debug output
        print("\nTest 1: _calculate_bearing(lat, lon, target_lat, target_lon)")
        bearing1 = calculate_bearing(lat, lon, target_lat, target_lon)
        
        # Swapped target coordinates
        print("\nTest 2: _calculate_bearing(lat, lon, target_lon, target_lat)")
        bearing2 = calculate_bearing(lat, lon, target_lon, target_lat)
        
        # Swapped current coordinates
        print("\nTest 3: _calculate_bearing(lon, lat, target_lat, target_lon)")
        bearing3 = calculate_bearing(lon, lat, target_lat, target_lon)
        
        # Stanley controller method (based on your code)
        print("\nTest 4: _calculate_bearing(y=lat, x=lon, tx=target_lat, ty=target_lon)")
        bearing4 = calculate_bearing(lat, lon, target_lat, target_lon)
        
        # Display summary
        print("\n==== RESULTS SUMMARY ====")
        print(f"Current Position: ({lat:.7f}, {lon:.7f}), Heading: {heading:.1f}°")
        print(f"Target Waypoint {waypoint_idx}: ({target_lat:.7f}, {target_lon:.7f})")
        print(f"Map Visualizer Bearing: {bearing:.1f}°")
        print(f"Test 1 - Normal Order: {bearing1:.1f}°")
        print(f"Test 2 - Swapped Target: {bearing2:.1f}°")
        print(f"Test 3 - Swapped Current: {bearing3:.1f}°")
        print(f"Test 4 - Stanley Controller: {bearing4:.1f}°")
        
        # Calculate distance
        distance = geodesic((lat, lon), (target_lat, target_lon)).meters
        print(f"Distance to waypoint: {distance:.1f} meters")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    debug_stanley_control()