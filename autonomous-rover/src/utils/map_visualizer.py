#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/map_visualizer.py

import folium
import webbrowser
import time
import math
import threading
import json
import sys
import os
import numpy as np
import http.server
import socketserver
import argparse
import logging
from datetime import datetime
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class MapVisualizer:
    def __init__(self, waypoints_file, port=8000):
        """Initialize the map visualizer with waypoints."""
        self.port = port
        self.waypoints = self._load_waypoints(waypoints_file)
        self.map_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rover_map.html')
        self.position_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'current_position.json')
        self.stop_event = threading.Event()
        
        # Create initial position file
        with open(self.position_file, 'w') as f:
            json.dump({
                "lat": self.waypoints[0][0],
                "lon": self.waypoints[0][1],
                "heading": 0,
                "target_idx": 0,
                "timestamp": datetime.now().isoformat()
            }, f)
        
        # Create the map
        self._create_map()
        
    def _load_waypoints(self, waypoints_file):
        """Load waypoints from file."""
        try:
            with open(waypoints_file, 'r') as f:
                if waypoints_file.endswith('.json'):
                    # JSON format
                    data = json.load(f)
                    if isinstance(data, list):
                        if len(data) > 0 and all('lat' in p and 'lon' in p for p in data):
                            # Format where each point is {lat: X, lon: Y}
                            waypoints = [(p['lat'], p['lon']) for p in data]
                            logger.info("Loaded waypoints from JSON with lat/lon keys")
                        else:
                            # Try other JSON formats
                            if isinstance(data, dict) and 'waypoints' in data:
                                waypoints = [(p['lat'], p['lon']) for p in data['waypoints']]
                                logger.info("Loaded waypoints from JSON waypoints array")
                            else:
                                # Try to extract as array of coordinates
                                try:
                                    # Determine format by checking first point
                                    if len(data) > 0:
                                        first_point = data[0]
                                        if isinstance(first_point, list) and len(first_point) >= 2:
                                            # Format is [[lat, lon], ...]
                                            waypoints = [(p[0], p[1]) for p in data]
                                            logger.info("Loaded waypoints from JSON array")
                                        else:
                                            # Unknown format
                                            waypoints = []
                                    else:
                                        waypoints = []
                                except:
                                    waypoints = []
                                    logger.error("Could not determine JSON format")
                else:
                    # Assume simple lat,lon format
                    waypoints = []
                    for line in f.readlines():
                        try:
                            parts = line.strip().split(',')
                            if len(parts) >= 2:
                                lat = float(parts[0])
                                lon = float(parts[1])
                                waypoints.append((lat, lon))
                        except ValueError:
                            continue
            
            logger.info(f"Loaded {len(waypoints)} waypoints")
            
            # Validate waypoint format - latitude should typically be between -90 and 90
            # while longitude is between -180 and 180
            if waypoints:
                sample_lat, sample_lon = waypoints[0]
                if abs(sample_lat) > 90 or abs(sample_lon) > 180:
                    logger.warning("Waypoint coordinates might be in incorrect format!")
                
                # Check if coordinates might be swapped
                if abs(sample_lat) < abs(sample_lon):
                    logger.warning("Latitude and longitude might be swapped in waypoints file!")
                    # This would mean waypoints are stored as (lon, lat) which is incorrect
                    # Let's fix it by swapping them back
                    waypoints = [(lon, lat) for lat, lon in waypoints]
                    logger.info("Automatically swapped coordinates to (lat, lon) format")
                    
            return waypoints
            
        except Exception as e:
            logger.error(f"Error loading waypoints: {e}")
            return []
            
    def _create_map(self):
        """Create the OpenStreetMap with waypoints and information panel."""
        if not self.waypoints:
            logger.error("No waypoints to visualize")
            return
            
        # Calculate map center
        center_lat = np.mean([wp[0] for wp in self.waypoints])
        center_lon = np.mean([wp[1] for wp in self.waypoints])
        
        # Create map
        m = folium.Map(location=[center_lat, center_lon], zoom_start=19)
        
        # Add zoom control
        folium.map.FitBounds(self.waypoints).add_to(m)

        # Add fullscreen control
        from folium.plugins import Fullscreen
        Fullscreen().add_to(m)

        # Add measure tool
        from folium.plugins import MeasureControl
        m.add_child(MeasureControl())
        
        # Create a feature group for waypoints to keep them separate
        waypoints_group = folium.FeatureGroup(name="Waypoints")
        m.add_child(waypoints_group)
        
        # Add waypoints to their own group
        for i, (lat, lon) in enumerate(self.waypoints):
            folium.Marker(
                [lat, lon],
                popup=f"Waypoint {i}",
                icon=folium.Icon(color="blue", icon="flag"),
            ).add_to(waypoints_group)
            
        # Connect waypoints with a line
        folium.PolyLine(
            self.waypoints,
            color="blue",
            weight=2.5,
            opacity=0.8,
        ).add_to(waypoints_group)
        
        # IMPORTANT: Remove the static rover marker
        # The dynamic one will be added by JavaScript
        
        # Add layers control so user can toggle waypoints if needed
        folium.LayerControl().add_to(m)
        
        # Add info panel for waypoint distances and headings (will be updated by JavaScript)
        info_panel = """
        <div id="info-panel" style="
            position: absolute;
            top: 10px;
            right: 10px;
            z-index: 1000;
            background-color: white;
            padding: 10px;
            border-radius: 5px;
            box-shadow: 0 0 10px rgba(0,0,0,0.5);
            max-width: 300px;
            max-height: 80%;
            overflow-y: auto;
        ">
            <h4 style="margin-top: 0;">Waypoint Information</h4>
            <div id="waypoint-info">Loading...</div>
        </div>
        """
        
        # Add the info panel to the map
        m.get_root().html.add_child(folium.Element(info_panel))
        
        # Add JavaScript to periodically update the rover position and info panel
        import json
        js_template = """
<script type="text/javascript">
// Global variables for map components
var roverMarker = null;
var roverLabel = null;  // Store label reference separately
var headingLine = null;
var waypoints = WAYPOINTS_JSON_PLACEHOLDER;
var mapFound = false;
var initAttempts = 0;

// More comprehensive map finder
function findMap() {
    console.log("Finding map, attempt " + (++initAttempts));
    
    // Method 1: Direct DOM query
    var mapContainers = document.querySelectorAll('.folium-map, #map, .leaflet-container');
    for (var i = 0; i < mapContainers.length; i++) {
        if (mapContainers[i]._leaflet_map) {
            console.log("Found map via DOM selector");
            return mapContainers[i]._leaflet_map;
        }
    }
    
    // Method 2: Search in window object
    for (var key in window) {
        if (window[key] && 
            typeof window[key] === 'object' && 
            window[key].getContainer && 
            typeof window[key].addLayer === 'function') {
            console.log("Found map in window object:", key);
            return window[key];
        }
    }
    
    // Method 3: Look for any element with _leaflet properties
    var allElements = document.getElementsByTagName('*');
    for (var i = 0; i < allElements.length; i++) {
        if (allElements[i]._leaflet_id || allElements[i]._leaflet_map) {
            console.log("Found leaflet element:", allElements[i]);
            return allElements[i]._leaflet_map || window.L.map(allElements[i]);
        }
    }
    
    console.log("Map not found");
    return null;
}

function updatePosition() {
    // Try to find the map if we haven't already
    var map = findMap();
    
    if (!map) {
        if (initAttempts > 20) {
            console.error("Failed to find map after " + initAttempts + " attempts");
            alert("Map initialization failed. Try reloading the page.");
            return;
        }
        setTimeout(updatePosition, 500);
        return;
    }
    
    // Only log once when map is found
    if (!mapFound) {
        console.log("Map found successfully!");
        mapFound = true;
    }

    fetch('current_position.json?' + new Date().getTime())
        .then(response => response.json())
        .then(data => {
            console.log("Position update received:", data);
            var latlng = [data.lat, data.lon];
            
            // Create or update rover marker
            if (!roverMarker) {
                console.log("Creating rover marker at:", latlng);
                
                // Create a simple, reliable marker first
                roverMarker = L.marker(latlng, {
                    icon: L.icon({
                        iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
                        iconSize: [25, 41],
                        iconAnchor: [12, 41]
                    })
                }).addTo(map);
                
                // Auto-center on marker when first found
                map.setView(latlng, 19);
                
                console.log("Rover marker created successfully");
            } else {
                roverMarker.setLatLng(latlng);
            }
            
            // Add popup with info
            var popupContent = "ROVER<br>Heading: " + data.heading.toFixed(1) + "°<br>" +
                              "Position: " + data.lat.toFixed(6) + ", " + data.lon.toFixed(6);
            roverMarker.bindPopup(popupContent);
            
            // Continue update loop
            setTimeout(updatePosition, 500);
        })
        .catch(error => {
            console.error("Error fetching position:", error);
            setTimeout(updatePosition, 2000);
        });
}

// Start the initialization process
document.addEventListener('DOMContentLoaded', function() {
    console.log("DOM loaded, starting map detection process");
    setTimeout(updatePosition, 1000);
});
</script>
"""
        
        # Replace the waypoints placeholder with the actual JSON data
        js = js_template.replace('WAYPOINTS_JSON_PLACEHOLDER', json.dumps(self.waypoints))
        
        # Add custom JavaScript to the map
        m.get_root().html.add_child(folium.Element(js))
        
        # Save the map
        m.save(self.map_file)
        logger.info(f"Map created at {self.map_file}")
        
    def update_position(self, lat, lon, heading, target_idx):
        """Update the current position of the rover."""
        with open(self.position_file, 'w') as f:
            json.dump({
                "lat": lat,
                "lon": lon,
                "heading": heading,
                "target_idx": target_idx,
                "timestamp": datetime.now().isoformat()
            }, f)
            
    def start_server(self):
        """Start the HTTP server to serve the map."""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        class CustomHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=current_dir, **kwargs)
                
            def log_message(self, format, *args):
                # Suppress server logs
                pass
        
        # Create and start the server in a separate thread
        self.server = socketserver.TCPServer(("", self.port), CustomHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
        logger.info(f"Server started at http://localhost:{self.port}")
        
        # Open the map in browser
        webbrowser.open(f"http://localhost:{self.port}/rover_map.html")
        
    def stop_server(self):
        """Stop the HTTP server."""
        if hasattr(self, 'server'):
            self.server.shutdown()
            self.server.server_close()
            logger.info("Server stopped")
            
    def __del__(self):
        """Clean up resources when the object is destroyed."""
        self.stop_server()
        
    def print_waypoint_info(self, current_lat=None, current_lon=None, current_heading=None):
        """Print static waypoint information to console"""
        from geopy.distance import geodesic
        
        # Use first waypoint as current position if not provided
        if current_lat is None or current_lon is None:
            # Try to read the current position from the position file
            try:
                with open(self.position_file, 'r') as f:
                    pos_data = json.load(f)
                    current_lat = pos_data.get('lat')
                    current_lon = pos_data.get('lon')
                    if current_heading is None:
                        current_heading = pos_data.get('heading', 0)
            except Exception:
                # Fall back to first waypoint
                if self.waypoints:
                    current_lat, current_lon = self.waypoints[0]
                else:
                    logger.warning("No waypoints available to print information")
                    return
        
        print("\n=== WAYPOINT INFORMATION ===")
        print(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
        print("-" * 40)
        
        for idx, (wp_lat, wp_lon) in enumerate(self.waypoints):
            # Calculate distance using geodesic
            distance = geodesic((current_lat, current_lon), (wp_lat, wp_lon)).meters
            
            # Calculate bearing using our consistent method
            bearing = self.calculate_bearing(current_lat, current_lon, wp_lat, wp_lon)
            
            # Calculate turn direction if heading is provided
            turn = ""
            if current_heading is not None:
                relative_bearing = bearing - current_heading
                relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
                
                if abs(relative_bearing) < 1:
                    turn_direction = "STRAIGHT"
                elif relative_bearing > 0:
                    turn_direction = "RIGHT"
                    turn = f"→ {abs(relative_bearing):.1f}°"
                else:
                    turn_direction = "LEFT"
                    turn = f"← {abs(relative_bearing):.1f}°"
            
            print(f"{idx:<4}{distance:<14.1f}{bearing:<10.1f}°{turn}")
        
        print("=" * 40)

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing between two points using the same algorithm as serial_gps_monitor.
        Returns bearing in degrees (0-360).
        """
        # Convert to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing
        dlon = lon2_rad - lon1_rad
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = math.degrees(bearing)
        bearing_deg = (bearing_deg + 360) % 360
        
        return bearing_deg

    def debug_bearing_calculations(self, lat1, lon1, lat2, lon2):
        """Debug function to compare bearing calculations"""
        try:
            from geopy.distance import geodesic
            
            # Calculate bearing using our method
            our_bearing = self.calculate_bearing(lat1, lon1, lat2, lon2)
            
            # Calculate distance
            distance = geodesic((lat1, lon1), (lat2, lon2)).meters
            
            # Calculate bearing using geopy - This was the problematic line
            # GeoPy's geodesic doesn't have a direct 'bearing' attribute
            # Instead, we need to calculate the initial bearing (forward azimuth)
            from geopy import Point
            from geopy.distance import distance as geopy_distance
            
            p1 = Point(lat1, lon1)
            p2 = Point(lat2, lon2)
            
            # Use the geodesic distance object to calculate bearing (azimuth)
            d = geopy_distance(p1, p2)
            geopy_bearing = d.initial_bearing
            if geopy_bearing < 0:
                geopy_bearing += 360  # Normalize to 0-360
        except ImportError:
            print("GeoPy not installed. Install with: pip install geopy")
            # Calculate using our own methods
            our_bearing = self.calculate_bearing(lat1, lon1, lat2, lon2)
            
            # Simple distance calculation using Haversine formula
            import math
            R = 6371000  # Earth radius in meters
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            dLat = lat2_rad - lat1_rad
            dLon = lon2_rad - lon1_rad
            a = math.sin(dLat/2) * math.sin(dLat/2) + \
                math.cos(lat1_rad) * math.cos(lat2_rad) * \
                math.sin(dLon/2) * math.sin(dLon/2)
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
            distance = R * c
            
            # Can't calculate geopy_bearing without geopy
            geopy_bearing = "N/A - GeoPy not installed"
        except Exception as e:
            print(f"Error during bearing calculation: {e}")
            our_bearing = self.calculate_bearing(lat1, lon1, lat2, lon2)
            distance = 0
            geopy_bearing = "Error"
        
        print("\n=== BEARING CALCULATION DEBUG ===")
        print(f"From: ({lat1:.7f}, {lon1:.7f})")
        print(f"To:   ({lat2:.7f}, {lon2:.7f})")
        print(f"Distance: {distance:.2f} meters")
        print(f"Our bearing calculation: {our_bearing:.2f}°")
        
        # Test inverse bearing
        inverse_bearing = self.calculate_bearing(lat2, lon2, lat1, lon1)
        print(f"Inverse bearing: {inverse_bearing:.2f}°")
        print(f"Normalized opposite: {(our_bearing + 180) % 360:.2f}°")
        
        # Test with swapped parameters
        print(f"With swapped lat/lon (From): {self.calculate_bearing(lon1, lat1, lat2, lon2):.2f}°")
        print(f"With swapped lat/lon (To): {self.calculate_bearing(lat1, lon1, lon2, lat2):.2f}°")
        print(f"With both swapped: {self.calculate_bearing(lon1, lat1, lon2, lat2):.2f}°")
        
        # Check waypoint ordering
        print("\nWaypoint format check:")
        if isinstance(self.waypoints[0], tuple) and len(self.waypoints[0]) == 2:
            first_wp = self.waypoints[0]
            print(f"First waypoint format: ({first_wp[0]:.7f}, {first_wp[1]:.7f})")
            print(f"Expected format: (latitude, longitude)")
            if abs(first_wp[0]) > abs(first_wp[1]):
                print("✓ Format appears to be (latitude, longitude)")
            else:
                print("⚠️ Format might be (longitude, latitude) - check this!")
        
        # Print GeoPy bearing if available
        if isinstance(geopy_bearing, (int, float)):
            print(f"\nGeoPy bearing: {geopy_bearing:.2f}°")
        else:
            print(f"\nGeoPy bearing: {geopy_bearing}")
        
        # Visual representation for easy checking
        directions = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        direction_idx = int(((our_bearing + 22.5) % 360) / 45)
        print(f"Direction: {directions[direction_idx]}")
        
        # Directional analysis
        if abs(lon2 - lon1) < 0.0000001:  # Nearly identical longitudes
            if lat2 > lat1:
                expected = "North (0°)"
            else:
                expected = "South (180°)"
        elif abs(lat2 - lat1) < 0.0000001:  # Nearly identical latitudes
            if lon2 > lon1:
                expected = "East (90°)"
            else:
                expected = "West (270°)"
        else:
            # Determine quadrant
            if lat2 > lat1 and lon2 > lon1:
                expected = "Northeast (0°-90°)"
            elif lat2 > lat1 and lon2 < lon1:
                expected = "Northwest (270°-360°)"
            elif lat2 < lat1 and lon2 > lon1:
                expected = "Southeast (90°-180°)"
            else:  # lat2 < lat1 and lon2 < lon1
                expected = "Southwest (180°-270°)"
        
        print(f"Expected direction: {expected}")
        
        # Display all bearings in a visual way  
        print("\nBearing comparison:")
        print(" " * 14 + "N")
        print(" " * 14 + "0°")
        print(f"W 270° {'-' * 10}•{'-' * 10} 90° E")
        print(" " * 14 + "180°")
        print(" " * 14 + "S")
        print(f"Our bearing: {our_bearing:.1f}° {'*' * int(our_bearing/10)}")
        
        # For direct west comparison
        if lat1 == 34.1519056 and lon1 == -77.8667716 and lat2 == 34.1519056 and lon2 == -77.8670785:
            print(f"Expected (due west): 270.0° {'*' * 27}")
        elif lat1 == current_lat and lon1 == current_lon:
            print(f"GPS test: 282.0° {'*' * 28}")
        
        print("=" * 40)

    def save_waypoints(self, filename=None):
        """Save waypoints to file in the correct format."""
        if filename is None:
            # Use the original filename with _fixed appended
            base_name = os.path.basename(self.waypoints_file)
            name, ext = os.path.splitext(base_name)
            filename = os.path.join(os.path.dirname(self.waypoints_file), f"{name}_fixed{ext}")
        
        try:
            # Save as JSON with lat/lon keys for clarity
            formatted_waypoints = [{"lat": wp[0], "lon": wp[1]} for wp in self.waypoints]
            with open(filename, 'w') as f:
                json.dump(formatted_waypoints, f, indent=2)
            logger.info(f"Waypoints saved to {filename}")
        except Exception as e:
            logger.error(f"Error saving waypoints: {e}")

def main():
    """Main function to run the visualizer as a standalone tool."""
    parser = argparse.ArgumentParser(description='Rover Map Visualizer')
    parser.add_argument('--waypoints', '-w', help='Path to waypoints file (default: data/polygon_data.json)',
                       default=None)
    parser.add_argument('--port', '-p', type=int, default=8000, help='HTTP server port')
    parser.add_argument('--debug', '-d', action='store_true', help='Run bearing calculation debug')
    args = parser.parse_args()
    
    # If waypoints file not specified, try to use the default polygon_data.json
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
            if (os.path.exists(path)):
                args.waypoints = path
                logger.info(f"Using default waypoints file: {path}")
                break
        
        if args.waypoints is None:
            logger.error("Default waypoints file not found. Please specify a waypoints file.")
            sys.exit(1)
    
    visualizer = MapVisualizer(args.waypoints, args.port)
    
    # Run debug if requested
    if args.debug:
        # Use hardcoded values from your test
        current_lat = 34.1519056
        current_lon = -77.8667716
        wp1_lat = 34.1519056
        wp1_lon = -77.8670785
        
        visualizer.debug_bearing_calculations(current_lat, current_lon, wp1_lat, wp1_lon)
        
        # Also test with actual GPS test coordinates and expected bearing of 279.5°
        print("\nTesting with actual GPS test coordinates:")
        visualizer.debug_bearing_calculations(current_lat, current_lon, 
                                             visualizer.waypoints[1][0], 
                                             visualizer.waypoints[1][1])
        
        # Analyze loaded waypoints 
        print("\nAnalyzing loaded waypoints:")
        for i, wp in enumerate(visualizer.waypoints):
            print(f"Waypoint {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
            
        return
    
    visualizer.print_waypoint_info()
    visualizer.start_server()  # Uncomment to start the server
    
    try:
        logger.info("Map visualizer running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping visualizer")
    finally:
        visualizer.stop_server()
            
if __name__ == "__main__":
    main()
