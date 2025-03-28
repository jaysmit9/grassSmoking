#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/static_map_visualizer.py

import folium
import json
import os
import time
import logging
import argparse
import http.server
import socketserver
import threading
import webbrowser
import numpy as np
from geopy.distance import geodesic

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("static_map_visualizer")

class StaticMapVisualizer:
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

    def create_map(self, rover_position=None):
        """Create a static map showing waypoints and current rover position."""
        if not self.waypoints:
            logger.error("No waypoints to visualize")
            return False
            
        # Calculate map center
        center_lat = np.mean([wp[0] for wp in self.waypoints])
        center_lon = np.mean([wp[1] for wp in self.waypoints])
        
        # Create map with Google Satellite as the default base layer
        m = folium.Map(
            location=[center_lat, center_lon], 
            zoom_start=19,
            max_zoom=21,
            tiles=None,  # No default tiles
            control_scale=True
        )
        
        # Add Google Satellite as the default (first) layer
        google_satellite = folium.TileLayer(
            tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            attr='Google Satellite',
            name='Google Satellite',
            overlay=False,
            max_zoom=21
        )
        google_satellite.add_to(m)
        
        # Add OpenStreetMap as alternative
        folium.TileLayer(
            tiles='https://tile.openstreetmap.org/{z}/{x}/{y}.png',
            attr='OpenStreetMap',
            name='OpenStreetMap',
            overlay=False,
            max_zoom=19
        ).add_to(m)
        
        # Add ESRI Satellite as another alternative
        folium.TileLayer(
            tiles='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            attr='Esri World Imagery',
            name='ESRI Satellite',
            overlay=False,
            max_zoom=21
        ).add_to(m)
        
        # Add layer control to switch between tile providers
        folium.LayerControl().add_to(m)
        
        # Add zoom home button to reset to overview
        from folium.plugins import LocateControl
        LocateControl().add_to(m)

        # Add fullscreen option
        from folium.plugins import Fullscreen
        Fullscreen().add_to(m)
        
        # Calculate distances if rover position is available
        distances = None
        if rover_position:
            distances = self.calculate_distances(rover_position)
        
        # Add waypoints with distance information
        for i, (lat, lon) in enumerate(self.waypoints):
            # Prepare popup content
            popup_content = f"<b>Waypoint {i}</b>"
            
            # Add distance info if available
            if distances:
                # Find this waypoint in distances
                wp_dist = next((d for d in distances if d['index'] == i), None)
                if wp_dist:
                    distance_text = f"{wp_dist['distance']:.1f}m"
                    popup_content += f"<br>Distance: {wp_dist['distance']:.2f} m"
                    popup_content += f"<br>Bearing: {wp_dist['bearing']:.1f}°"
                    
                    # Add a distance text marker near the waypoint
                    folium.Marker(
                        [lat, lon],
                        icon=folium.DivIcon(
                            icon_size=(150, 36),
                            icon_anchor=(75, 0),
                            html=f'<div style="font-size: 12pt; background-color: rgba(255,255,255,0.7); padding: 3px; border-radius: 3px;">{distance_text}</div>'
                        )
                    ).add_to(m)
            
            # Create icon - highlight closest waypoint
            is_closest = distances and distances[0]['index'] == i
            icon_color = "green" if is_closest else "blue"
            icon_type = "star" if is_closest else "flag"
            
            # Create marker
            folium.Marker(
                [lat, lon],
                popup=popup_content,
                tooltip=f"WP {i}: {distance_text if distances else ''}",
                icon=folium.Icon(color=icon_color, icon=icon_type),
            ).add_to(m)
        
        # Connect waypoints with a line
        folium.PolyLine(
            self.waypoints,
            color="blue",
            weight=2.5,
            opacity=0.8,
        ).add_to(m)
        
        # Add rover position marker if available
        if rover_position:
            lat, lon, heading = rover_position
            
            # Create rover popup with detailed information
            popup_content = f"""
                <b>ROVER</b><br>
                Heading: {heading:.1f}°<br>
                Position: {lat:.6f}, {lon:.6f}<br>
            """
            
            # Add info about closest waypoint
            if distances and len(distances) > 0:
                next_wp = distances[0]
                popup_content += f"""
                    <hr>
                    <b>Closest waypoint: {next_wp['index']}</b><br>
                    Distance: {next_wp['distance']:.2f} m<br>
                    Bearing: {next_wp['bearing']:.1f}°<br>
                    Rel. Heading: {((next_wp['bearing'] - heading + 360) % 360):.1f}°
                """
            
            # Add a background circle for better visibility on satellite imagery
            folium.CircleMarker(
                location=[lat, lon],
                radius=15,
                color='black',
                weight=2,
                fill=True,
                fill_color='yellow',
                fill_opacity=0.5,
                popup="Rover position background"
            ).add_to(m)
            
            # Create rover marker with a more visible icon
            folium.Marker(
                [lat, lon],
                popup=folium.Popup(popup_content, max_width=300),
                tooltip="ROVER",
                icon=folium.Icon(
                    color="red",
                    icon="crosshairs",  # More visible than "car"
                    prefix="fa",
                    icon_color="white"
                ),
            ).add_to(m)
            
            # Add a line from rover to closest waypoint
            if distances and len(distances) > 0:
                next_wp = distances[0]
                folium.PolyLine(
                    [[lat, lon], next_wp['waypoint']],
                    color="green",
                    weight=3,
                    dash_array="5",
                    opacity=0.7,
                    popup=f"Distance: {next_wp['distance']:.2f} m"
                ).add_to(m)
            
            # Zoom to rover position
            m.location = [lat, lon]
        
        # Add a distance scale
        folium.plugins.MeasureControl(position='bottomright', primary_length_unit='meters').add_to(m)
        
        # Auto-fit the map bounds to show all waypoints
        bounds = [(wp[0], wp[1]) for wp in self.waypoints]
        if rover_position:
            bounds.append((rover_position[0], rover_position[1]))
        m.fit_bounds(bounds)
        
        # Add a distance summary panel if rover position is available
        if rover_position and distances:
            lat, lon, heading = rover_position
            
            # Create HTML for the distance panel
            distance_html = """
            <div style="position: fixed; 
                        bottom: 10px; 
                        left: 10px; 
                        z-index: 1000; 
                        background-color: white; 
                        padding: 10px; 
                        border-radius: 5px; 
                        box-shadow: 0 0 5px rgba(0,0,0,0.3);
                        max-height: 300px;
                        overflow-y: auto;">
                <h4>Distances to Waypoints</h4>
                <table style="width: 100%;">
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
                rel_heading = (d['bearing'] - heading + 360) % 360
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
        
        # Add auto-refresh meta tag for periodic updates
        meta_refresh = """
        <meta http-equiv="refresh" content="5">
        <div style="position:fixed; bottom:10px; left:10px; background:rgba(255,255,255,0.7); padding:5px; border-radius:5px;">Auto-refreshing every 5 seconds</div>
        """
        m.get_root().header.add_child(folium.Element(meta_refresh))
        
        # Save map
        self.save_map(m)
        return True
        
    def save_map(self, m):
        """Save the map with improvements for visibility."""
        # Add CSS to ensure map takes full height
        css = """
        <style>
        html, body, #map {
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
        }
        .rover-marker {
            z-index: 1000 !important;  /* Ensure rover is on top */
        }
        </style>
        """
        m.get_root().header.add_child(folium.Element(css))
        
        # Save the map
        m.save(self.map_file)
        logger.info(f"Created map at {self.map_file}")

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

    def calculate_distances(self, rover_position):
        """Calculate distances from rover to all waypoints."""
        if not rover_position:
            return None
            
        rover_lat, rover_lon, _ = rover_position
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
        return distances

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing from point 1 to point 2."""
        import math
        
        # Convert to radians
        lat1, lon1 = math.radians(lat1), math.radians(lon1)
        lat2, lon2 = math.radians(lat2), math.radians(lon2)
        
        # Calculate bearing
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.degrees(math.atan2(y, x))
        
        # Normalize to 0-360
        return (bearing + 360) % 360

def main():
    """Main function to run the static map visualizer."""
    parser = argparse.ArgumentParser(description='Static Rover Map Visualizer')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file')
    parser.add_argument('--lat', type=float, help='Rover latitude')
    parser.add_argument('--lon', type=float, help='Rover longitude')
    parser.add_argument('--heading', type=float, default=0, help='Rover heading in degrees')
    parser.add_argument('--port', '-p', type=int, default=8000, help='HTTP server port')
    args = parser.parse_args()
    
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
    visualizer = StaticMapVisualizer(args.waypoints, args.port)
    
    # Get rover position if provided
    rover_position = None
    if args.lat is not None and args.lon is not None:
        rover_position = (args.lat, args.lon, args.heading)
        
    # Create map
    if visualizer.create_map(rover_position):
        visualizer.start_server()
        visualizer.open_browser()
        
        try:
            logger.info("Press Ctrl+C to exit")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Exiting...")
        finally:
            visualizer.stop_server()
    
if __name__ == "__main__":
    main()