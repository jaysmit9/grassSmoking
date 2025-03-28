#!/usr/bin/env python3

import time
import math
import threading
import logging
import requests
import json
from geopy.distance import geodesic

# Configure logging
logger = logging.getLogger("rover.hardware.gps")

class GPSMonitor:
    """
    GPS monitor for tracking position and heading using data from a GPS API endpoint.
    
    This implementation provides the same interface as the serial version but gets
    data from a HTTP API instead of serial ports.
    """
    
    def __init__(self, api_url='http://localhost:5001/api/gps', timeout=0.5, 
                 update_interval=0.1, simulation_mode=False):
        """
        Initialize GPS monitor that gets data from an API endpoint.
        
        Args:
            api_url: URL to the GPS API endpoint
            timeout: API request timeout in seconds
            update_interval: How often to update GPS data in seconds
            simulation_mode: If True, generate simulated GPS data
        """
        self.simulation_mode = simulation_mode
        self.api_url = api_url
        self.timeout = timeout
        self.update_interval = update_interval
        
        # Initialize GPS data structure
        self.gps_data = {
            "front": self._create_empty_gps_data(),
            "rear": self._create_empty_gps_data()
        }
        
        logger.info(f"Initializing API GPS Monitor (api={api_url}, simulation={simulation_mode})")
        
        # For real hardware mode
        if not simulation_mode:
            try:
                # Test API connection
                response = requests.get(self.api_url, timeout=self.timeout)
                if response.status_code == 200:
                    logger.info("Successfully connected to GPS API")
                    # Start the API poller thread
                    self._start_api_poller()
                else:
                    logger.error(f"Failed to connect to GPS API: {response.status_code}")
                    logger.warning("Switching to simulation mode")
                    self.simulation_mode = True
            except Exception as e:
                logger.error(f"Failed to connect to GPS API: {e}")
                logger.warning("Switching to simulation mode")
                self.simulation_mode = True
        
        # Start simulation if needed
        if self.simulation_mode:
            self._start_simulation()
        
        # Initialize last status update time
        self._last_status_log = 0
        
        # Initialize constants
        self.EARTH_RADIUS = 6371000  # Earth's radius in meters
    
    def _create_empty_gps_data(self):
        """Create an empty GPS data structure"""
        return {
            "lat": None,
            "lon": None, 
            "heading": None,
            "speed": 0.0,
            "fix_quality": 0,
            "last_update": 0,
            "messages_per_sec": 0,
            "hdop": 99.9,
            "satellites": 0
        }
    
    def _start_api_poller(self):
        """Start thread to poll the GPS API"""
        self.running = True
        
        # Create and start API poller thread
        self.api_thread = threading.Thread(
            target=self._api_poller,
            daemon=True
        )
        self.api_thread.start()
    
    def _api_poller(self):
        """Thread function to continuously poll GPS API"""
        logger.debug(f"Starting GPS API poller thread")
        last_update_time = time.time()
        
        try:
            while self.running:
                try:
                    # Get data from API
                    response = requests.get(self.api_url, timeout=self.timeout)
                    if response.status_code == 200:
                        api_data = response.json()
                        self._process_api_data(api_data)
                    else:
                        logger.warning(f"GPS API returned status {response.status_code}")
                    
                    # Calculate and update message rate
                    now = time.time()
                    if now - last_update_time >= 1.0:
                        # Just set to a fixed update rate since we're polling
                        self.gps_data["front"]["messages_per_sec"] = 1 / self.update_interval
                        self.gps_data["rear"]["messages_per_sec"] = 1 / self.update_interval
                        last_update_time = now
                    
                    # Wait for next update
                    time.sleep(self.update_interval)
                    
                except requests.RequestException as e:
                    logger.error(f"Error in GPS API request: {e}")
                    time.sleep(1)  # Avoid rapid error loops
                    
                except Exception as e:
                    logger.error(f"Error in GPS API poller: {e}")
                    time.sleep(1)  # Avoid rapid error loops
                    
        except Exception as e:
            logger.error(f"Fatal error in GPS API poller thread: {e}")
    
    def _process_api_data(self, api_data):
        """Process GPS data from the API response"""
        try:
            # Extract front GPS data
            front_device = api_data.get('devices', {}).get('Front GPS', {})
            if front_device:
                self.gps_data["front"]["lat"] = front_device.get('lat')
                self.gps_data["front"]["lon"] = front_device.get('lon')
                self.gps_data["front"]["heading"] = front_device.get('track')
                self.gps_data["front"]["speed"] = front_device.get('speed', 0.0)
                self.gps_data["front"]["fix_quality"] = front_device.get('gga_quality', 0)
                self.gps_data["front"]["hdop"] = front_device.get('hdop', 99.9)
                self.gps_data["front"]["satellites"] = front_device.get('satellites_used', 0)
                self.gps_data["front"]["last_update"] = front_device.get('last_update', time.time())
            
            # Extract rear GPS data
            rear_device = api_data.get('devices', {}).get('Rear GPS', {})
            if rear_device:
                self.gps_data["rear"]["lat"] = rear_device.get('lat')
                self.gps_data["rear"]["lon"] = rear_device.get('lon')
                self.gps_data["rear"]["heading"] = rear_device.get('track')
                self.gps_data["rear"]["speed"] = rear_device.get('speed', 0.0)
                self.gps_data["rear"]["fix_quality"] = rear_device.get('gga_quality', 0)
                self.gps_data["rear"]["hdop"] = rear_device.get('hdop', 99.9)
                self.gps_data["rear"]["satellites"] = rear_device.get('satellites_used', 0)
                self.gps_data["rear"]["last_update"] = rear_device.get('last_update', time.time())
            
            # Log status periodically
            now = time.time()
            if logger.isEnabledFor(logging.DEBUG) and now - self._last_status_log > 5.0:
                self._log_gps_status()
                self._last_status_log = now
                
        except Exception as e:
            logger.error(f"Error processing API GPS data: {e}")
    
    def _start_simulation(self):
        """Create a simple GPS simulation"""
        logger.info("Starting GPS simulation")
        
        # Default starting position in Wilmington
        self.gps_data["front"]["lat"] = 34.2257
        self.gps_data["front"]["lon"] = -77.9447
        self.gps_data["front"]["heading"] = 0.0
        self.gps_data["front"]["speed"] = 0.0
        self.gps_data["front"]["fix_quality"] = 4
        self.gps_data["front"]["satellites"] = 14
        self.gps_data["front"]["hdop"] = 0.8
        self.gps_data["front"]["last_update"] = time.time()
        
        # Rear GPS is 30cm behind
        self.gps_data["rear"]["lat"] = 34.2257 - 0.0000027
        self.gps_data["rear"]["lon"] = -77.9447
        self.gps_data["rear"]["heading"] = 0.0
        self.gps_data["rear"]["speed"] = 0.0
        self.gps_data["rear"]["fix_quality"] = 4
        self.gps_data["rear"]["satellites"] = 14
        self.gps_data["rear"]["hdop"] = 0.8
        self.gps_data["rear"]["last_update"] = time.time()
        
        # Set running flag
        self.running = True
        
        # Start simulation thread
        self.sim_thread = threading.Thread(
            target=self._sim_thread_function,
            daemon=True
        )
        self.sim_thread.start()
    
    def _sim_thread_function(self):
        """Run GPS simulation updates"""
        # Constants for simulation
        update_interval = 0.1      # 10Hz update rate
        
        # Simulation state
        sim_speed = 0.0
        sim_heading = 0.0
        last_update = time.time()
        
        try:
            while self.running:
                now = time.time()
                delta_t = now - last_update
                
                if delta_t >= update_interval:
                    last_update = now
                    
                    # Simulate movement
                    distance = sim_speed * delta_t  # meters
                    
                    if distance > 0:
                        # Convert heading to radians for calculation
                        heading_rad = math.radians(sim_heading)
                        
                        # Get current position
                        front_lat = self.gps_data["front"]["lat"]
                        front_lon = self.gps_data["front"]["lon"]
                        
                        # Calculate new position using geodesic approach
                        d = distance / 1000.0  # Convert to kilometers for this calculation
                        R = 6371.0  # Earth's radius in km
                        
                        lat1 = math.radians(front_lat)
                        lon1 = math.radians(front_lon)
                        
                        # Calculate new position
                        lat2 = math.asin(math.sin(lat1) * math.cos(d/R) +
                                         math.cos(lat1) * math.sin(d/R) * math.cos(heading_rad))
                        lon2 = lon1 + math.atan2(math.sin(heading_rad) * math.sin(d/R) * math.cos(lat1),
                                               math.cos(d/R) - math.sin(lat1) * math.sin(lat2))
                        
                        # Convert back to degrees
                        new_lat = math.degrees(lat2)
                        new_lon = math.degrees(lon2)
                        
                        # Update front GPS position
                        self.gps_data["front"]["lat"] = new_lat
                        self.gps_data["front"]["lon"] = new_lon
                    
                    # Always update these properties
                    self.gps_data["front"]["heading"] = sim_heading
                    self.gps_data["front"]["speed"] = sim_speed
                    self.gps_data["front"]["last_update"] = now
                    self.gps_data["front"]["messages_per_sec"] = 10.0
                    
                    # Calculate rear GPS position (30cm behind front)
                    rear_heading_rad = math.radians((sim_heading + 180) % 360)
                    rear_distance = 0.3 / 1000.0  # 30cm in kilometers
                    
                    # Front is our origin now
                    lat1 = math.radians(self.gps_data["front"]["lat"])
                    lon1 = math.radians(self.gps_data["front"]["lon"])
                    
                    # Calculate rear position
                    R = 6371.0  # Earth's radius in km
                    lat2 = math.asin(math.sin(lat1) * math.cos(rear_distance/R) +
                                    math.cos(lat1) * math.sin(rear_distance/R) * math.cos(rear_heading_rad))
                    lon2 = lon1 + math.atan2(math.sin(rear_heading_rad) * math.sin(rear_distance/R) * math.cos(lat1),
                                           math.cos(rear_distance/R) - math.sin(lat1) * math.sin(lat2))
                    
                    # Convert back to degrees
                    rear_lat = math.degrees(lat2)
                    rear_lon = math.degrees(lon2)
                    
                    # Update rear GPS position
                    self.gps_data["rear"]["lat"] = rear_lat
                    self.gps_data["rear"]["lon"] = rear_lon
                    self.gps_data["rear"]["heading"] = sim_heading
                    self.gps_data["rear"]["speed"] = sim_speed
                    self.gps_data["rear"]["last_update"] = now
                    self.gps_data["rear"]["messages_per_sec"] = 10.0
                
                # Periodically log status if in debug mode
                if logger.isEnabledFor(logging.DEBUG) and now - self._last_status_log > 5.0:
                    self._log_gps_status()
                    self._last_status_log = now
                
                # Sleep briefly to avoid CPU hogging
                time.sleep(0.01)
                
        except Exception as e:
            logger.error(f"Error in GPS simulation: {e}")
    
    def _log_gps_status(self):
        """Log GPS status for debugging"""
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        logger.debug("=== GPS STATUS ===")
        logger.debug(f"Front: ({front['lat']:.7f}, {front['lon']:.7f}) hdg={front['heading']}° spd={front['speed']:.1f}m/s fix={front['fix_quality']} sat={front['satellites']}")
        logger.debug(f"Rear: ({rear['lat']:.7f}, {rear['lon']:.7f}) hdg={rear['heading']}° spd={rear['speed']:.1f}m/s fix={rear['fix_quality']} sat={rear['satellites']}")
        
        # Calculate distances between GPSes
        if front["lat"] is not None and rear["lat"] is not None:
            distance = self.calculate_distance((rear["lat"], rear["lon"]), (front["lat"], front["lon"]))
            logger.debug(f"Distance between GPS units: {distance:.3f}m")
            
            # Calculate bearing using our method
            bearing = self.calculate_bearing(rear["lat"], rear["lon"], front["lat"], front["lon"])
            logger.debug(f"Calculated bearing between GPS units: {bearing:.1f}°")
        
        logger.debug("=================")
    
    def get_position_and_heading(self):
        """Get the current position (lat, lon) and heading from the GPS devices"""
        # In simulation mode, return simulated data
        if self.simulation_mode:
            front = self.gps_data["front"]
            rear = self.gps_data["rear"]
            
            # Calculate heading between GPSes
            if None not in (front["lat"], front["lon"], rear["lat"], rear["lon"]):
                calculated_heading = self.calculate_bearing(
                    rear["lat"], rear["lon"],
                    front["lat"], front["lon"]
                )
            else:
                calculated_heading = front["heading"]
                
            return front["lat"], front["lon"], calculated_heading, front["speed"]
        
        # For API mode, use the GPS data we've collected
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        lat, lon = None, None
        heading = None
        speed = 0.0
        
        # First, try to get position from front GPS
        if front["lat"] is not None and front["lon"] is not None:
            lat = front["lat"]
            lon = front["lon"]
            speed = front["speed"] if front["speed"] is not None else 0.0
            
            # Try to get heading from the dual-GPS setup - either from API or calculate ourselves
            if rear["lat"] is not None and rear["lon"] is not None:
                # Try to get API-provided bearing from dual GPS
                try:
                    # Make a fresh API request to get latest bearing
                    response = requests.get(self.api_url, timeout=self.timeout)
                    if response.status_code == 200:
                        api_data = response.json()
                        dual_gps = api_data.get('dual_gps', {})
                        if dual_gps and 'bearing' in dual_gps:
                            heading = dual_gps['bearing']
                            logger.debug("Using API-provided dual GPS bearing")
                except:
                    pass
                
                # If we couldn't get it from API, calculate it ourselves
                if heading is None:
                    heading = self.calculate_bearing(
                        rear["lat"], rear["lon"],
                        front["lat"], front["lon"]
                    )
                    logger.debug("Heading calculated from dual GPS positions")
            
            # If dual-GPS heading failed, try using heading data from API
            if heading is None and front["heading"] is not None:
                heading = front["heading"]
                logger.debug("Using heading from front GPS")
        
        # If heading is still not available, try using previously stored positions
        if heading is None and hasattr(self, '_last_positions') and len(self._last_positions) >= 2:
            try:
                # Calculate heading from last two positions
                prev_lat, prev_lon, _ = self._last_positions[-2]
                curr_lat, curr_lon, _ = self._last_positions[-1]
                
                # Only calculate if we've moved enough
                min_movement = 0.5  # meters
                dist = self.calculate_distance((prev_lat, prev_lon), (curr_lat, curr_lon))
                
                if dist > min_movement:
                    movement_heading = self.calculate_bearing(prev_lat, prev_lon, curr_lat, curr_lon)
                    heading = movement_heading
                    logger.debug(f"Using movement-based heading: {heading:.1f}°")
            except Exception as e:
                logger.warning(f"Error calculating movement-based heading: {e}")
        
        # Store position history for movement-based heading
        timestamp = time.time()
        if not hasattr(self, '_last_positions'):
            self._last_positions = []
        
        if lat is not None and lon is not None:
            self._last_positions.append((lat, lon, timestamp))
            # Keep last 10 positions
            if len(self._last_positions) > 10:
                self._last_positions.pop(0)
        
        # Log warning if heading is still not available
        if heading is None:
            logger.warning("No heading available (missing GPS data)")
        
        return lat, lon, heading, speed
    
    def calculate_distance(self, point1, point2):
        """
        Calculate the distance between two points using geodesic formula.
        
        Args:
            point1: Tuple of (lat, lon) for first point
            point2: Tuple of (lat, lon) for second point
            
        Returns:
            float: Distance in meters
        """
        try:
            return geodesic(point1, point2).meters
        except Exception as e:
            logger.error(f"Error calculating distance: {e}")
            return 0.0
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing between two points.
        
        Args:
            lat1, lon1: Source coordinates (decimal degrees)
            lat2, lon2: Target coordinates (decimal degrees)
            
        Returns:
            float: Bearing in degrees (0-360 range)
        """
        try:
            # Convert decimal degrees to radians
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            # Calculate bearing
            dlon = lon2_rad - lon1_rad
            y = math.sin(dlon) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
            bearing_rad = math.atan2(y, x)
            
            # Convert to degrees and normalize to 0-360
            bearing_deg = math.degrees(bearing_rad)
            bearing_deg = (bearing_deg + 360) % 360
            
            return bearing_deg
        except Exception as e:
            logger.error(f"Error calculating bearing: {e}")
            return 0.0
    
    def get_distance_to_waypoint(self, waypoint):
        """
        Calculate the geodesic distance from current position to a waypoint.
        
        Args:
            waypoint: Tuple of (lat, lon) coordinates
            
        Returns:
            float: Distance in meters
        """
        # Get current position
        current_lat = self.gps_data["front"]["lat"]
        current_lon = self.gps_data["front"]["lon"]
        
        # Check if we have valid position
        if current_lat is None or current_lon is None:
            logger.warning("Cannot calculate distance: No valid GPS position")
            return 0.0
            
        # Calculate distance using geodesic formula
        try:
            distance = geodesic((current_lat, current_lon), waypoint).meters
            return distance
        except Exception as e:
            logger.error(f"Error calculating distance to waypoint: {e}")
            return 0.0
    
    def get_bearing_to_waypoint(self, waypoint):
        """
        Calculate the bearing from current position to a waypoint.
        
        Args:
            waypoint: Tuple of (lat, lon) coordinates
            
        Returns:
            float: Bearing in degrees (0-360 range)
        """
        # Get current position
        current_lat = self.gps_data["front"]["lat"]
        current_lon = self.gps_data["front"]["lon"]
        
        # Check if we have valid position
        if current_lat is None or current_lon is None:
            logger.warning("Cannot calculate bearing: No valid GPS position")
            return 0.0
            
        # Calculate bearing
        return self.calculate_bearing(current_lat, current_lon, waypoint[0], waypoint[1])
    
    def get_gps_info(self):
        """Get detailed GPS information for debugging"""
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        # Try to get API-provided bearing (if available)
        calculated_heading = None
        try:
            response = requests.get(self.api_url, timeout=self.timeout)
            if response.status_code == 200:
                api_data = response.json()
                dual_gps = api_data.get('dual_gps', {})
                if dual_gps and 'bearing' in dual_gps:
                    calculated_heading = dual_gps['bearing']
        except:
            pass
            
        # If not available from API, calculate it ourselves
        if calculated_heading is None and None not in (rear["lat"], rear["lon"], front["lat"], front["lon"]):
            calculated_heading = self.calculate_bearing(
                rear["lat"], rear["lon"],
                front["lat"], front["lon"]
            )
        
        info = {
            "front": {
                "position": (front["lat"], front["lon"]),
                "heading": front["heading"],
                "speed": front["speed"],
                "fix": front["fix_quality"],
                "hdop": front["hdop"],
                "satellites": front["satellites"],
                "update_rate": front["messages_per_sec"],
                "last_update": front["last_update"]
            },
            "rear": {
                "position": (rear["lat"], rear["lon"]),
                "heading": rear["heading"],
                "speed": rear["speed"], 
                "fix": rear["fix_quality"],
                "hdop": rear["hdop"],
                "satellites": rear["satellites"],
                "update_rate": rear["messages_per_sec"],
                "last_update": rear["last_update"]
            },
            "rover": {
                "position": (front["lat"], front["lon"]),
                "calculated_heading": calculated_heading,
                "reported_heading": front["heading"],
                "speed": front["speed"]
            }
        }
        
        return info
    
    def stop(self):
        """Stop GPS monitor and release resources"""
        self.running = False
    
    def debug_bearing_from_positions(self):
        """Debug method to print bearing calculations with current positions"""
        front_lat = self.gps_data["front"]["lat"]
        front_lon = self.gps_data["front"]["lon"]
        rear_lat = self.gps_data["rear"]["lat"]
        rear_lon = self.gps_data["rear"]["lon"]
        
        # Check if we have valid positions before calculating
        if None in (front_lat, front_lon, rear_lat, rear_lon):
            logger.warning("Cannot debug bearing: Some GPS positions are missing")
            logger.warning(f"Front GPS: {front_lat}, {front_lon}")
            logger.warning(f"Rear GPS: {rear_lat}, {rear_lon}")
            return None
        
        # Check if API provides bearing
        api_bearing = None
        try:
            response = requests.get(self.api_url, timeout=self.timeout)
            if response.status_code == 200:
                api_data = response.json()
                dual_gps = api_data.get('dual_gps', {})
                if dual_gps and 'bearing' in dual_gps:
                    api_bearing = dual_gps['bearing']
        except:
            pass
        
        # Calculate bearing from rear to front GPS
        calculated_bearing = self.calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
        
        # Get the heading from GPS module
        reported_heading = self.gps_data["front"]["heading"]
        
        # Print debug info
        logger.info("===== BEARING DEBUG =====")
        logger.info(f"Front GPS: ({front_lat:.7f}, {front_lon:.7f})")
        logger.info(f"Rear GPS: ({rear_lat:.7f}, {rear_lon:.7f})")
        logger.info(f"API-provided bearing: {api_bearing:.1f}°" if api_bearing is not None else "API-provided bearing: N/A")
        logger.info(f"Calculated bearing: {calculated_bearing:.1f}°")
        
        # Calculate distance between GPS units
        distance = self.calculate_distance((rear_lat, rear_lon), (front_lat, front_lon))
        logger.info(f"Distance between GPS units: {distance:.3f}m")
        
        # Check if reported heading is available
        if reported_heading is not None:
            logger.info(f"Reported heading: {reported_heading:.1f}° (from GPS)")
            logger.info(f"Difference: {abs(calculated_bearing - reported_heading):.1f}°")
        else:
            logger.info("Reported heading: Not available")
            
        logger.info("========================")
        
        # Return the API bearing if available, otherwise our calculated one
        return api_bearing if api_bearing is not None else calculated_bearing
    
    def debug_distance_calculation(self, lat, lon, target_wp):
        """
        Debug distance calculation between points.
        
        Args:
            lat, lon: Source coordinates
            target_wp: Tuple of (lat, lon) for destination
        """
        logger.info("\n=== DISTANCE CALCULATION DEBUG ===")
        logger.info(f"From: ({lat:.7f}, {lon:.7f})")
        logger.info(f"To:   ({target_wp[0]:.7f}, {target_wp[1]:.7f})")
        
        # Calculate geodesic distance
        geodesic_distance = self.calculate_distance((lat, lon), target_wp)
        logger.info(f"Geodesic distance: {geodesic_distance:.2f} meters")
        
        # Calculate bearing
        bearing = self.calculate_bearing(lat, lon, target_wp[0], target_wp[1])
        logger.info(f"Bearing: {bearing:.1f}°")
        
        logger.info("=================================")
        
        return geodesic_distance, bearing

    def get_distance_between_gps(self):
        """
        Calculate the distance between front and rear GPS units.
        
        Returns:
            float: Distance in meters
        """
        # Try to get from API first
        try:
            response = requests.get(self.api_url, timeout=self.timeout)
            if response.status_code == 200:
                api_data = response.json()
                dual_gps = api_data.get('dual_gps', {})
                if dual_gps and 'distance' in dual_gps:
                    return dual_gps['distance']
        except:
            pass
            
        # Fall back to calculating ourselves
        front_lat = self.gps_data["front"]["lat"]
        front_lon = self.gps_data["front"]["lon"]
        rear_lat = self.gps_data["rear"]["lat"]
        rear_lon = self.gps_data["rear"]["lon"]
        
        # Check if we have valid positions before calculating
        if None in (front_lat, front_lon, rear_lat, rear_lon):
            logger.warning("Cannot calculate GPS separation: Some GPS positions are missing")
            return None
        
        # Calculate distance between GPS units
        distance = self.calculate_distance((rear_lat, rear_lon), (front_lat, front_lon))
        return distance


def initialize_gps_with_retry(max_attempts=3):
    """Initialize GPS with retry mechanism"""
    for attempt in range(max_attempts):
        try:
            logger.info(f"Initializing GPS (attempt {attempt+1}/{max_attempts})...")
            gps = GPSMonitor()
            
            # Test if GPS is working
            lat, lon, heading, _ = gps.get_position_and_heading()
            if lat is not None and lon is not None:
                logger.info("GPS initialized successfully")
                return gps
                
            logger.warning("GPS returned None values, retrying...")
            time.sleep(1)
            
        except Exception as e:
            logger.error(f"Error initializing GPS: {e}")
            
            # Try to close any existing connections if we have them
            try:
                if 'gps' in locals() and gps is not None:
                    gps.stop()
            except:
                pass
                
            time.sleep(1)
    
    # If we reach here, we failed to initialize GPS
    logger.error("Failed to initialize GPS after multiple attempts")
    return None


# Test function for the GPS monitor
def test_gps_monitor():
    """Test the GPS monitor functionality"""
    logging.basicConfig(level=logging.INFO, 
                        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    
    print("Testing API GPS Monitor...")
    monitor = GPSMonitor()
    
    # Test API connection
    print("\nGetting position and heading...")
    lat, lon, heading, speed = monitor.get_position_and_heading()
    print(f"Position: ({lat}, {lon})")
    print(f"Heading: {heading}")
    print(f"Speed: {speed} m/s")
    
    # Test GPS info
    print("\nGetting GPS info...")
    info = monitor.get_gps_info()
    print(f"Front GPS: {info['front']['position']}")
    print(f"Rear GPS: {info['rear']['position']}")
    print(f"Rover heading: {info['rover']['calculated_heading']}")
    
    # Test distance between GPS units
    print("\nGetting distance between GPS units...")
    distance = monitor.get_distance_between_gps()
    print(f"Distance: {distance} meters")
    
    # Clean up
    monitor.stop()
    print("Test complete")


if __name__ == "__main__":
    test_gps_monitor()