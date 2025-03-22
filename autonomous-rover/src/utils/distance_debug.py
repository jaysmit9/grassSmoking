#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/distance_debug.py

import os
import sys
import json
import logging
from geopy.distance import geodesic

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Add the project root to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import necessary modules
try:
    from hardware.gps_monitor import GPSMonitor
    from main import load_waypoints, run_gps_test
except ImportError as e:
    logger.error(f"Error importing modules: {e}")
    logger.error("Make sure you're running this from the correct directory.")
    sys.exit(1)

# Add this near the top of your file, after the imports
class NumpyEncoder(json.JSONEncoder):
    """Special JSON encoder for numpy types"""
    def default(self, obj):
        if isinstance(obj, np.integer):
            return int(obj)
        elif isinstance(obj, np.floating):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

def find_waypoints_file():
    """Find the polygon_data.json file"""
    possible_paths = [
        'data/polygon_data.json',
        '../data/polygon_data.json',
        '../../data/polygon_data.json',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/polygon_data.json'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/polygon_data.json')
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            logger.info(f"Found waypoints file: {path}")
            return path
    
    logger.error("Could not find waypoints file!")
    return None

def print_raw_waypoints(waypoints_file):
    """Print the raw JSON content of the waypoints file"""
    logger.info("=== RAW WAYPOINTS FILE CONTENT ===")
    
    try:
        with open(waypoints_file, 'r') as f:
            content = json.load(f)
            
        if isinstance(content, list):
            logger.info(f"File contains a list with {len(content)} items")
            if content:
                logger.info(f"First item: {content[0]}")
        elif isinstance(content, dict):
            logger.info(f"File contains a dictionary with keys: {list(content.keys())}")
        else:
            logger.info(f"File contains data of type: {type(content)}")
            
    except Exception as e:
        logger.error(f"Error reading waypoints file: {e}")

# Update test_direct_calculation function to handle NumPy arrays
def test_direct_calculation(waypoints_file):
    """Test distance calculation directly"""
    logger.info("\n=== DIRECT CALCULATION TEST ===")
    
    # Test position - same as used previously
    test_pos = (34.1519056, -77.8667716)
    logger.info(f"Test position: {test_pos}")
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Fix for NumPy arrays
    import numpy as np
    if isinstance(waypoints, np.ndarray):
        logger.info(f"Loaded waypoints as NumPy array with shape {waypoints.shape}")
        
        if waypoints.size == 0:
            logger.error("No waypoints loaded (empty array)!")
            return
    else:
        # Original check for lists
        if not waypoints:
            logger.error("No waypoints loaded!")
            return
        
    # Get the number of waypoints
    if isinstance(waypoints, np.ndarray):
        num_waypoints = len(waypoints) if waypoints.ndim > 0 else 0
    else:
        num_waypoints = len(waypoints)
        
    logger.info(f"Loaded {num_waypoints} waypoints")
    
    # Calculate distances using geodesic
    logger.info("\nGeodetic distances:")
    
    # Limit to first 4 waypoints or fewer if there are less
    max_waypoints = min(4, num_waypoints)
    
    for i in range(max_waypoints):
        wp = waypoints[i]
        
        # Handle differently based on type
        if isinstance(wp, np.ndarray):
            # Convert NumPy array to tuple for geodesic
            wp_tuple = tuple(wp)
        else:
            wp_tuple = wp
            
        dist = geodesic(test_pos, wp_tuple).meters
        logger.info(f"WP {i}: {wp} => {dist:.1f} meters")
        
    logger.info(f"Expected GPS test distances: 12.0, 18.8, 13.1, 2.1")

def inspect_run_gps_test():
    """Inspect and analyze the run_gps_test function"""
    logger.info("\n=== GPS TEST FUNCTION ANALYSIS ===")
    
    # Get the source code of the run_gps_test function
    import inspect
    try:
        source = inspect.getsource(run_gps_test)
        logger.info("Function source code:")
        for line in source.split('\n'):
            if "dist" in line and "geodesic" in line:
                logger.info(f"DISTANCE CALCULATION: {line.strip()}")
            elif "dist" in line and "=" in line:
                logger.info(f"POSSIBLE DISTANCE CALCULATION: {line.strip()}")
    except Exception as e:
        logger.error(f"Could not get source: {e}")

def modify_waypoints_for_test(waypoints_file, output_file):
    """Create a modified waypoints file that would match the GPS test distances"""
    logger.info("\n=== CREATING MODIFIED WAYPOINTS FILE ===")
    
    # Test position
    test_pos = (34.1519056, -77.8667716)
    
    # Load original waypoints
    original_waypoints = load_waypoints(waypoints_file)
    
    # Handle NumPy arrays
    import numpy as np
    if isinstance(original_waypoints, np.ndarray):
        if original_waypoints.size == 0:
            logger.error("No waypoints loaded (empty array)!")
            return
        # Continue with array
        waypoints_count = len(original_waypoints)
    else:
        if not original_waypoints:
            logger.error("No waypoints loaded!")
            return
        waypoints_count = len(original_waypoints)
    
    logger.info(f"Loaded {waypoints_count} original waypoints")
    
    # Expected distances from GPS test
    expected_distances = [12.0, 18.8, 13.1, 2.1]
    
    # Calculate scaling factors
    scaling_factors = []
    for i in range(min(len(expected_distances), waypoints_count)):
        wp = original_waypoints[i]
        
        # Convert to tuple if necessary
        if isinstance(wp, np.ndarray):
            wp_tuple = tuple(wp)
        else:
            wp_tuple = wp
            
        actual_dist = geodesic(test_pos, wp_tuple).meters
        if actual_dist > 0:
            scaling_factors.append(expected_distances[i] / actual_dist)
            logger.info(f"WP {i}: Expected={expected_distances[i]}m, Actual={actual_dist:.1f}m, Ratio={expected_distances[i]/actual_dist:.3f}")
        else:
            scaling_factors.append(0)
    
    # Calculate average scaling factor
    if scaling_factors:
        avg_scale = sum(scaling_factors) / len(scaling_factors)
        logger.info(f"Average scaling factor: {avg_scale:.3f}")
        
        # Create modified waypoints
        modified_waypoints = []
        
        for i in range(waypoints_count):
            wp = original_waypoints[i]
            
            # Convert to regular Python types if NumPy
            if isinstance(wp, np.ndarray):
                wp = tuple(wp)
                
            if i < len(expected_distances):
                # Apply scaling
                lat_diff = wp[0] - test_pos[0]
                lon_diff = wp[1] - test_pos[1]
                
                new_lat = test_pos[0] + lat_diff * avg_scale
                new_lon = test_pos[1] + lon_diff * avg_scale
                modified_waypoints.append((new_lat, new_lon))
            else:
                # Keep as is
                modified_waypoints.append(wp)
        
        # Save modified waypoints
        try:
            with open(output_file, 'w') as f:
                json.dump(modified_waypoints, f, cls=NumpyEncoder)
            logger.info(f"Modified waypoints saved to: {output_file}")
        except Exception as e:
            logger.error(f"Error saving modified waypoints: {e}")
    else:
        logger.error("Could not calculate scaling factors")

def examine_run_gps_test(waypoints_file):
    """Run the GPS test function with debug output"""
    logger.info("\n=== DIRECT EXAMINATION OF GPS TEST ===")
    
    # Create a modified version of run_gps_test that adds debug info
    try:
        # Get the source of the run_gps_test function
        import inspect
        source = inspect.getsource(run_gps_test)
        
        # Save the source to a file for reference
        debug_file = os.path.join(os.path.dirname(waypoints_file), 'gps_test_source.txt')
        with open(debug_file, 'w') as f:
            f.write(source)
            
        logger.info(f"Saved run_gps_test source to: {debug_file}")
        logger.info("Check this file to see the exact distance calculation method used")
        
        # Look for distance calculation patterns
        if "dist = geodesic" in source:
            logger.info("Found geodesic distance calculation in run_gps_test")
        else:
            logger.info("Did not find direct geodesic calculation in run_gps_test")
            
            # Look for other patterns that might indicate distance calculations
            if "dist =" in source:
                import re
                dist_calcs = re.findall(r'dist\s*=\s*[^;]+', source)
                if dist_calcs:
                    logger.info("Found these distance calculations:")
                    for calc in dist_calcs:
                        logger.info(f"  {calc.strip()}")
                        
    except Exception as e:
        logger.error(f"Error examining run_gps_test: {e}")

def main():
    # Find the waypoints file
    waypoints_file = find_waypoints_file()
    if not waypoints_file:
        return
    
    # Print the raw waypoints content
    print_raw_waypoints(waypoints_file)
    
    # Test direct calculation
    test_direct_calculation(waypoints_file)
    
    # Inspect the run_gps_test function
    inspect_run_gps_test()
    
    # Add this new call
    examine_run_gps_test(waypoints_file)
    
    # Create a modified waypoints file
    modified_file = os.path.join(os.path.dirname(waypoints_file), 'modified_polygon_data.json')
    modify_waypoints_for_test(waypoints_file, modified_file)
    
    logger.info("\n=== NEXT STEPS ===")
    logger.info(f"1. Try running --gps-test with the modified waypoints file:")
    logger.info(f"   python main.py --gps-test --waypoints={modified_file}")
    logger.info(f"2. Check if the distances now match your expectations")
    logger.info(f"3. In main.py, look for the actual distance calculation in run_gps_test")
    logger.info(f"   and verify if it's using the geodesic method correctly")

if __name__ == "__main__":
    main()