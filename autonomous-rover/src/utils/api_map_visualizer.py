#!/usr/bin/env python3

import os
import sys
import time
import json
import threading
import argparse
import logging
import requests
from datetime import datetime
from pathlib import Path

# Import the MapVisualizer class
from map_visualizer import MapVisualizer

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("rover.api_visualizer")

class ApiMapVisualizer:
    """
    Map visualizer that gets position data from the GPS API endpoint.
    """
    
    def __init__(self, waypoints_file, api_url="http://localhost:5001/api/gps", 
                 update_interval=0.5, port=8000):
        """
        Initialize the API map visualizer.
        
        Args:
            waypoints_file: Path to the waypoints JSON file
            api_url: URL of the GPS API endpoint
            update_interval: How often to fetch position data (seconds)
            port: HTTP server port for the map
        """
        self.api_url = api_url
        self.update_interval = update_interval
        self.running = False
        self.last_error_time = 0
        self.error_count = 0
        
        # Create the map visualizer
        logger.info(f"Initializing map with waypoints from {waypoints_file}")
        self.visualizer = MapVisualizer(waypoints_file, port)
        
        # Try an initial API request
        try:
            logger.info(f"Testing connection to API at {api_url}")
            self._fetch_gps_data()
            logger.info("Successfully connected to GPS API")
        except Exception as e:
            logger.warning(f"Could not connect to GPS API: {e}")
            logger.warning("Visualization will start but position updates may be delayed")
    
    def _fetch_gps_data(self):
        """
        Fetch GPS data from the API endpoint.
        
        Returns:
            dict: GPS data containing lat, lon, heading
        """
        try:
            response = requests.get(self.api_url, timeout=2.0)
            if response.status_code == 200:
                return response.json()
            else:
                logger.warning(f"API returned error status: {response.status_code}")
                return None
        except Exception as e:
            # Limit error logging to avoid spamming
            current_time = time.time()
            if current_time - self.last_error_time > 10:  # Log errors at most every 10 seconds
                logger.error(f"Error fetching GPS data: {e}")
                self.last_error_time = current_time
                self.error_count = 1
            else:
                self.error_count += 1
                if self.error_count % 10 == 0:
                    logger.warning(f"Continued API connection errors ({self.error_count} in the last 10 seconds)")
            return None
    
    def _get_current_position(self):
        """
        Extract position data from the API response.
        
        Returns:
            tuple: (lat, lon, heading, target_idx)
        """
        gps_data = self._fetch_gps_data()
        if not gps_data:
            return None
        
        try:
            # Extract front GPS position (primary position)
            front_device = gps_data.get('devices', {}).get('Front GPS', {})
            if front_device:
                lat = front_device.get('lat')
                lon = front_device.get('lon')
                
                # Get heading - first try dual_gps bearing if available
                heading = gps_data.get('dual_gps', {}).get('bearing')
                if heading is None:
                    # Fall back to front GPS track if available
                    heading = front_device.get('track')
                
                # If still no heading, try rear GPS
                if heading is None:
                    rear_device = gps_data.get('devices', {}).get('Rear GPS', {})
                    if rear_device:
                        heading = rear_device.get('track')
                
                # If still no heading, default to 0
                if heading is None:
                    heading = 0.0
                
                # For target_idx, we don't have this from the API
                # Use 0 as default or try to find closest waypoint
                target_idx = 0
                
                return (lat, lon, heading, target_idx)
            else:
                logger.warning("No front GPS data available")
                return None
        except Exception as e:
            logger.error(f"Error processing GPS data: {e}")
            return None
    
    def _update_loop(self):
        """Background thread for updating position from API."""
        while self.running:
            try:
                position = self._get_current_position()
                if position:
                    lat, lon, heading, target_idx = position
                    # Change from debug to info level temporarily for troubleshooting
                    logger.info(f"Updating position: lat={lat:.6f}, lon={lon:.6f}, heading={heading:.1f}°")
                    self.visualizer.update_position(lat, lon, heading, target_idx)
                else:
                    logger.info("No position data available from API")
                
                # Sleep for the update interval
                time.sleep(self.update_interval)
            except Exception as e:
                logger.error(f"Error in update loop: {e}")
                time.sleep(1.0)  # Longer sleep on error
    
    def _debug_api_data(self):
        """Temporary function to debug raw API data"""
        data = self._fetch_gps_data()
        if data:
            logger.info(f"Raw API data sample: {json.dumps(data, indent=2)[:500]}...")
        else:
            logger.warning("No data received from API")
        return data
    
    def start(self):
        """Start the visualizer and position updates."""
        # Debug API data
        self._debug_api_data()
        
        # Start the map server
        self.visualizer.update_position(0, 0, 0, 0)  # Reset position
        self.visualizer.start_server()
        
        # Start the update thread
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        logger.info("API Map Visualizer started")
        return self
    
    def stop(self):
        """Stop the visualizer and position updates."""
        self.running = False
        if hasattr(self, 'update_thread'):
            self.update_thread.join(timeout=1.0)
        
        # Stop the map server
        self.visualizer.stop_server()
        logger.info("API Map Visualizer stopped")

def find_waypoints_file():
    """Find the waypoints file in the standard locations."""
    possible_paths = [
        'data/polygon_data.json',
        '../data/polygon_data.json',
        '../../data/polygon_data.json',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../data/polygon_data.json'),
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '../data/polygon_data.json')
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            return path
    
    return None

def main():
    """Main function to run the API map visualizer."""
    parser = argparse.ArgumentParser(description='API-based Rover Map Visualizer')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file',
                        default=None)
    parser.add_argument('--api', '-a', help='GPS API endpoint URL',
                        default="http://localhost:5001/api/gps")
    parser.add_argument('--interval', '-i', type=float, default=0.5,
                        help='Update interval in seconds')
    parser.add_argument('--port', '-p', type=int, default=8000,
                        help='HTTP server port')
    args = parser.parse_args()
    
    # Find waypoints file if not specified
    if args.waypoints is None:
        waypoints_file = find_waypoints_file()
        if waypoints_file is None:
            logger.error("Could not find waypoints file. Please specify with --waypoints option.")
            sys.exit(1)
        args.waypoints = waypoints_file
    
    # Create and start the visualizer
    visualizer = ApiMapVisualizer(
        waypoints_file=args.waypoints,
        api_url=args.api,
        update_interval=1.0,  # Slower updates for testing
        port=args.port
    ).start()
    
    # Run until keyboard interrupt
    try:
        logger.info("API Map Visualizer running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping API Map Visualizer...")
    finally:
        visualizer.stop()

if __name__ == "__main__":
    main()