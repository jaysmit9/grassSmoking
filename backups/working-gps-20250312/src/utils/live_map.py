#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/live_map.py

import os
import sys
import time
import logging
import argparse
from datetime import datetime

# Add the src directory to the Python path
script_path = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.abspath(os.path.join(script_path, '..'))
sys.path.append(src_path)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

try:
    from hardware.gps_monitor import GPSMonitor
    from utils.map_vis import create_static_map, load_waypoints
except ImportError as e:
    logger.error(f"Error importing modules: {e}")
    sys.exit(1)

def continuous_map_update(waypoints_file, output_file="live_rover_map.html", update_interval=5):
    """Continuously update the map with the current GPS position."""
    # Initialize GPS
    try:
        gps = GPSMonitor()
        logger.info("GPS monitor initialized")
    except Exception as e:
        logger.error(f"Failed to initialize GPS: {e}")
        return
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    if not waypoints:
        logger.error("No waypoints loaded. Exiting.")
        return
    
    logger.info(f"Loaded {len(waypoints)} waypoints")
    logger.info(f"Map will update every {update_interval} seconds")
    logger.info("Press Ctrl+C to stop")
    
    try:
        while True:
            # Get current position and heading
            lat, lon, heading, speed = gps.get_position_and_heading()
            
            if lat is not None and lon is not None:
                # Update timestamp in filename to force browser refresh
                timestamp = datetime.now().strftime('%H%M%S')
                current_output = output_file.replace('.html', f'_{timestamp}.html')
                
                # Create map
                logger.info(f"Updating map with position: ({lat:.7f}, {lon:.7f})")
                create_static_map(waypoints, (lat, lon), heading, current_output)
                
                # Create a symlink or copy to the main file
                try:
                    # On Linux/Mac, create a symlink
                    if os.path.exists(output_file):
                        os.remove(output_file)
                    os.symlink(current_output, output_file)
                    logger.info(f"Updated map: {output_file} -> {current_output}")
                except:
                    # On Windows or if symlink fails, just copy the file
                    import shutil
                    shutil.copy2(current_output, output_file)
                    logger.info(f"Updated map: {output_file}")
                
                # Clean up old map files (keep only the last 5)
                map_files = [f for f in os.listdir('.') if f.startswith(output_file.replace('.html', '_')) and f.endswith('.html')]
                map_files.sort(reverse=True)
                for old_file in map_files[5:]:
                    try:
                        os.remove(old_file)
                    except:
                        pass
            else:
                logger.warning("No valid GPS position available")
            
            # Sleep until next update
            time.sleep(update_interval)
    
    except KeyboardInterrupt:
        logger.info("Map updates stopped by user")
    finally:
        # Clean up GPS resources
        gps.stop()

def main():
    parser = argparse.ArgumentParser(description='Live Map Updater for Rover')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file', default=None)
    parser.add_argument('--output', '-o', help='Output HTML file path', default="live_rover_map.html")
    parser.add_argument('--interval', '-i', type=int, default=5, help='Update interval in seconds')
    
    args = parser.parse_args()
    
    # If waypoints file not specified, try to find the default
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
    
    # Run the continuous map update
    continuous_map_update(args.waypoints, args.output, args.interval)

if __name__ == "__main__":
    main()