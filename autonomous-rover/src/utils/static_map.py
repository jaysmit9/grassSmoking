#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/simple_map.py

import folium
from folium import plugins
import json
import os
import time
import logging
import argparse
import http.server
import socketserver
import threading
import requests
import webbrowser
import numpy as np
from geopy.distance import geodesic
import math


# Setup logging
# Sample JSON data from the GPS API
#{"devices":{"Front GPS":{"alt":18.5,"baud_rate":115200,"connected":true,"gga_quality":4,"hdop":0.63,"is_front":true,"last_rates_time":1743080185.5140243,"last_rtcm_time":0,"last_update":1743080185.9176679,"lat":34.366714333333334,"lon":-78.22747766666667,"name":"Front GPS","port":"/dev/ttyACM1","rtcm_count":0,"rtk_mode":"RTK_FIX","satellites_used":12,"satellites_visible":1,"speed":0.004115552,"track":null,"update_rate":299.533592726208},"Rear GPS":{"alt":18.5,"baud_rate":115200,"connected":true,"gga_quality":1,"hdop":0.63,"is_front":false,"last_rates_time":1743080185.5140243,"last_rtcm_time":1743080184.4827106,"last_update":1743080185.8818161,"lat":34.366707166666664,"lon":-78.22748066666666,"name":"Rear GPS","port":"/dev/ttyACM0","rtcm_count":3399,"rtk_mode":"3D_FIX","satellites_used":12,"satellites_visible":1,"speed":null,"track":null,"update_rate":124.80566363592}},"dual_gps":{"bearing":19.061755078719898,"distance":0.843128122471561,"front_gps":{"lat":34.366714333333334,"lon":-78.22747766666667},"rear_gps":{"lat":34.366707166666664,"lon":-78.22748066666666}},"timestamp":1743080185.94442}
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("simple_map")

class SimpleMapVisualizer:
    def __init__(self, waypoints_file, port=8000):
        """Initialize the static map visualizer."""
        self.port = port
        self.waypoints = self._load_waypoints(waypoints_file)
        self.server = None
        self.server_thread = None
        self.map_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rover_map.html')
        
    def _load_waypoints(self, waypoints_file):
        """Load waypoints from file."""
        try:
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                
                # Handle different JSON formats
                if isinstance(data, list):
                    if len(data) > 0 and all('lat' in p and 'lon' in p for p in data):
                        waypoints = [(p['lat'], p['lon']) for p in data]
                    else:
                        # Try to handle as array of coordinates
                        waypoints = [(p[0], p[1]) for p in data]
                elif isinstance(data, dict) and 'waypoints' in data:
                    waypoints = [(p['lat'], p['lon']) for p in data['waypoints']]
                else:
                    waypoints = []
                    
            logger.info(f"Loaded {len(waypoints)} waypoints")
            return waypoints
        except Exception as e:
            logger.error(f"Error loading waypoints: {e}")
            return []
    
    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2."""
        # Convert to radians
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        # Calculate bearing
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.degrees(math.atan2(y, x))
        
        # Normalize to 0-360
        return (bearing + 360) % 360

    def create_map(self, rover_position, auto_refresh=True):
        """Create a static map showing waypoints and current rover position."""
        # ALWAYS require rover position - this ensures we show the rover
        if not rover_position:
            logger.error("Rover position required")
            return False
            
        # Ensure we have waypoints
        if not self.waypoints:
            logger.error("No waypoints to visualize")
            return False
            
        # Extract rover position
        rover_lat, rover_lon, rover_heading = rover_position
        logger.info(f"Creating map with rover at: {rover_lat}, {rover_lon}, heading: {rover_heading}")
        
        # Calculate map center - use rover position
        center_lat = rover_lat
        center_lon = rover_lon
        
        # Create map with Google Satellite as default
        m = folium.Map(
            location=[center_lat, center_lon], 
            zoom_start=20,  # Higher zoom level
            tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            attr='Google Satellite',
            max_zoom=21
        )
        
        # Add layer control
        folium.LayerControl().add_to(m)
        
        # Calculate distances from rover to waypoints
        distances = []
        for i, (wp_lat, wp_lon) in enumerate(self.waypoints):
            # Calculate distance using geodesic
            distance = geodesic((rover_lat, rover_lon), (wp_lat, wp_lon)).meters
            bearing = self._calculate_bearing(rover_lat, rover_lon, wp_lat, wp_lon)
            distances.append({
                'index': i,
                'distance': distance,
                'bearing': bearing,
                'waypoint': (wp_lat, wp_lon)
            })
        
        # Sort by distance
        distances.sort(key=lambda x: x['distance'])
        
        # Add waypoints with distance information
        for i, (wp_lat, wp_lon) in enumerate(self.waypoints):
            # Find this waypoint in distances
            wp_dist = next((d for d in distances if d['index'] == i), None)
            distance_text = f"{wp_dist['distance']:.1f}m" if wp_dist else ""
            
            # Create icon - highlight closest waypoint
            is_closest = distances and distances[0]['index'] == i
            icon_color = "green" if is_closest else "blue"
            icon_type = "star" if is_closest else "flag"
            
            # Add distance label
            if wp_dist:
                folium.Marker(
                    [wp_lat, wp_lon],
                    icon=folium.DivIcon(
                        icon_size=(150, 36),
                        icon_anchor=(75, 0),
                        html=f'<div style="font-size: 12pt; background-color: rgba(255,255,255,0.7); padding: 3px; border-radius: 3px;">{distance_text}</div>'
                    )
                ).add_to(m)
            
            # Create waypoint marker
            folium.Marker(
                [wp_lat, wp_lon],
                popup=f"<b>Waypoint {i}</b><br>Distance: {distance_text}",
                tooltip=f"WP {i}: {distance_text}",
                icon=folium.Icon(color=icon_color, icon=icon_type),
            ).add_to(m)
        
        # Connect waypoints with a line
        folium.PolyLine(
            self.waypoints,
            color="blue",
            weight=2.5,
            opacity=0.8,
        ).add_to(m)
        
        # GUARANTEED ROVER MARKER - always visible with high z-index
        # First add a background circle for better visibility
        folium.CircleMarker(
            location=[rover_lat, rover_lon],
            radius=8,
            color='black',
            weight=2,
            fill=True,
            fill_color='yellow',
            fill_opacity=1.0,
            popup="Rover position",
            tooltip="ROVER HERE"
        ).add_to(m)
        
        # Add a text label "ROVER" that's always visible
        folium.Marker(
            [rover_lat, rover_lon],
            icon=folium.DivIcon(
                icon_size=(100, 36),
                icon_anchor=(50, 18),
                html=f'<div style="font-size: 14pt; font-weight: bold; color: red; text-shadow: -1px -1px 0 #fff, 1px -1px 0 #fff, -1px 1px 0 #fff, 1px 1px 0 #fff;">ROVER</div>'
            )
        ).add_to(m)
        
        # Add a line from rover to closest waypoint
        if distances and len(distances) > 0:
            next_wp = distances[0]
            folium.PolyLine(
                [[rover_lat, rover_lon], next_wp['waypoint']],
                color="red",
                weight=3,
                opacity=0.7,
                popup=f"Distance: {next_wp['distance']:.2f} m"
            ).add_to(m)
        
        # Add a distance scale
        plugins.MeasureControl(position='bottomright', primary_length_unit='meters').add_to(m)
        
        # Auto-refresh
        if auto_refresh:
            auto_refresh_html = """
            <meta http-equiv="refresh" content="5">
            <div style="position:fixed; bottom:10px; left:10px; z-index:9999; background:rgba(255,255,255,0.8); 
                 padding:5px; border-radius:5px; font-weight:bold; color:#000;">
                 Auto-refresh enabled
            </div>
            """
            m.get_root().header.add_child(folium.Element(auto_refresh_html))
        
        # Add a distance summary panel
        if distances:
            distance_html = """
            <div style="position: fixed; 
                        top: 10px; 
                        right: 10px; 
                        z-index: 1000; 
                        background-color: white; 
                        padding: 10px; 
                        border-radius: 5px; 
                        box-shadow: 0 0 5px rgba(0,0,0,0.3);
                        max-height: 300px;
                        overflow-y: auto;">
                <h4>Distances to Waypoints</h4>
                <table border="1" style="width: 100%;">
                    <tr>
                        <th>WP</th>
                        <th>Distance</th>
                        <th>Bearing</th>
                        <th>Turn</th>
                    </tr>
            """
            
            # Add a row for each waypoint
            for d in distances:
                # Calculate turn direction
                rel_heading = (d['bearing'] - rover_heading + 360) % 360
                turn_dir = "←" if rel_heading > 180 else "→"
                turn_amount = min(rel_heading, 360 - rel_heading)
                
                # Add table row
                distance_html += f"""
                    <tr>
                        <td>{d['index']}</td>
                        <td>{d['distance']:.1f}m</td>
                        <td>{d['bearing']:.1f}°</td>
                        <td>{turn_dir}{turn_amount:.1f}°</td>
                    </tr>
                """
            
            distance_html += """
                </table>
            </div>
            """
            
            # Add the HTML panel to the map
            m.get_root().html.add_child(folium.Element(distance_html))
        
        # Save the map
        m.save(self.map_file)
        logger.info(f"Created map at {self.map_file}")
        return True

    def start_server(self):
        """Start a simple HTTP server to serve the map."""
        if self.server:
            logger.warning("Server already running")
            return
            
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        class CustomHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=current_dir, **kwargs)
                
            def log_message(self, format, *args):
                pass  # Suppress server logs
                
        self.server = socketserver.TCPServer(("", self.port), CustomHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        logger.info(f"Server started at http://localhost:{self.port}")
        
    def open_browser(self):
        """Open the map in a web browser."""
        webbrowser.open(f"http://localhost:{self.port}/rover_map.html")
        
    def stop_server(self):
        """Stop the HTTP server."""
        if self.server:
            self.server.shutdown()
            self.server.server_close()
            self.server = None
            self.server_thread = None
            logger.info("Server stopped")

def main():
    """Main function to run the static map visualizer."""
    parser = argparse.ArgumentParser(description='Simple Rover Map Visualizer')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file')
    parser.add_argument('--port', '-p', type=int, default=8000, help='HTTP server port')
    
    # Create position source group (either API or manual coordinates)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--api', '-a', action='store_true', help='Use GPS API for position')
    group.add_argument('--manual', '-m', action='store_true', help='Use manual coordinates')
    
    # Position arguments only required for manual mode
    parser.add_argument('--lat', type=float, help='Rover latitude (required with --manual)')
    parser.add_argument('--lon', type=float, help='Rover longitude (required with --manual)')
    parser.add_argument('--heading', type=float, default=0, help='Rover heading in degrees')
    parser.add_argument('--api-url', default="http://localhost:5001/api/gps", help='GPS API URL')
    
    # Add this to your parser arguments
    parser.add_argument('--no-refresh', action='store_true', help='Disable auto-refresh in browser')
    
    args = parser.parse_args()
    
    # Check for required arguments in manual mode
    if args.manual and (args.lat is None or args.lon is None):
        parser.error("--lat and --lon are required when using --manual mode")
    
    # Find waypoints file if not specified
    if not args.waypoints:
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
                logger.info(f"Using waypoints file: {path}")
                break
                
        if not args.waypoints:
            logger.error("No waypoints file found. Please specify with --waypoints")
            return
    
    # Create visualizer
    visualizer = SimpleMapVisualizer(args.waypoints, args.port)
    
    # Get rover position - either from API or command line args
    if args.api:
        try:
            response = requests.get(args.api_url, timeout=1.0)
            data = response.json()
            
            # Correctly parse the JSON structure from your API
            if "dual_gps" in data:
                # Use the dual_gps structure
                rover_lat = data["dual_gps"]["front_gps"]["lat"]
                rover_lon = data["dual_gps"]["front_gps"]["lon"]
                rover_heading = data["dual_gps"]["bearing"]
                logger.info(f"Got position from dual_gps: {rover_lat}, {rover_lon}, {rover_heading}")
            elif "devices" in data and "Front GPS" in data["devices"]:
                # Fall back to devices.Front GPS
                rover_lat = data["devices"]["Front GPS"]["lat"]
                rover_lon = data["devices"]["Front GPS"]["lon"]
                # Try to get heading from track if available
                rover_heading = data["devices"]["Front GPS"].get("track", 0)
                logger.info(f"Got position from Front GPS device: {rover_lat, rover_lon, rover_heading}")
            else:
                logger.error(f"Unexpected JSON structure: {data}")
                return
        except Exception as e:
            logger.error(f"Error getting position from API: {e}")
            return
    else:
        rover_lat = args.lat
        rover_lon = args.lon
        rover_heading = args.heading
    
    # Create map with rover position
    if visualizer.create_map((rover_lat, rover_lon, rover_heading), not args.no_refresh):
        visualizer.start_server()
        visualizer.open_browser()
        
        try:
            logger.info("Map server running. Press Ctrl+C to exit")
            while True:
                if args.api:
                    try:
                        response = requests.get(args.api_url, timeout=1.0)
                        data = response.json()
                        
                        # Use the same parsing logic as the initial position fetch
                        if "dual_gps" in data:
                            # Use the dual_gps structure
                            rover_lat = data["dual_gps"]["front_gps"]["lat"]
                            rover_lon = data["dual_gps"]["front_gps"]["lon"]
                            rover_heading = data["dual_gps"]["bearing"]
                        elif "devices" in data and "Front GPS" in data["devices"]:
                            # Fall back to devices.Front GPS
                            rover_lat = data["devices"]["Front GPS"]["lat"]
                            rover_lon = data["devices"]["Front GPS"]["lon"]
                            rover_heading = data["devices"]["Front GPS"].get("track", 0)
                        else:
                            logger.error(f"Unexpected JSON structure in update: {data}")
                            continue
                            
                        visualizer.create_map((rover_lat, rover_lon, rover_heading), not args.no_refresh)
                    except Exception as e:
                        logger.error(f"Error updating from API: {e}")
                time.sleep(4.5)  # Slightly less than refresh interval
        except KeyboardInterrupt:
            logger.info("Exiting...")
        finally:
            visualizer.stop_server()
    
if __name__ == "__main__":
    main()