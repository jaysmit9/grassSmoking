#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/hardware/gps_monitor.py

import time
import math
import threading
import logging
import serial
import numpy as np
from geopy.distance import geodesic

# Configure logging
logger = logging.getLogger("rover.hardware.gps")

class GPSMonitor:
    """
    GPS monitor for tracking position and heading using dual GPS modules.
    
    This implementation consistently uses geodesic calculations for distances
    and provides reliable bearing calculation between coordinates.
    """
    
    def __init__(self, front_port='/dev/ttyACM0', rear_port='/dev/ttyACM1', 
                 baud_rate=115200, timeout=0.1, simulation_mode=False):
        """
        Initialize GPS monitor that handles front and rear GPS units.
        
        Args:
            front_port: Serial port for front GPS
            rear_port: Serial port for rear GPS
            baud_rate: Baud rate for serial communication
            timeout: Serial timeout in seconds
            simulation_mode: If True, generate simulated GPS data
        """
        self.simulation_mode = simulation_mode
        self.gps_data = {
            "front": self._create_empty_gps_data(),
            "rear": self._create_empty_gps_data()
        }
        
        logger.info(f"Initializing GPS Monitor (simulation={simulation_mode})")
        
        # For real hardware mode
        if not simulation_mode:
            try:
                # Try to connect to GPS hardware
                self.front_serial = serial.Serial(front_port, baud_rate, timeout=timeout)
                self.rear_serial = serial.Serial(rear_port, baud_rate, timeout=timeout)
                logger.info(f"Connected to front GPS on {front_port}")
                logger.info(f"Connected to rear GPS on {rear_port}")
                
                # Start GPS reader threads
                self._start_gps_readers()
            except Exception as e:
                logger.error(f"Failed to initialize hardware GPS: {e}")
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
    
    def _start_gps_readers(self):
        """Start threads to read from the GPS hardware"""
        self.running = True
        
        # Create and start front GPS reader thread
        self.front_thread = threading.Thread(
            target=self._gps_reader,
            args=(self.front_serial, "front"),
            daemon=True
        )
        self.front_thread.start()
        
        # Create and start rear GPS reader thread
        self.rear_thread = threading.Thread(
            target=self._gps_reader,
            args=(self.rear_serial, "rear"),
            daemon=True
        )
        self.rear_thread.start()
    
    def _gps_reader(self, serial_port, gps_name):
        """Thread function to read and parse GPS data"""
        logger.debug(f"Starting {gps_name} GPS reader thread")
        message_buffer = ""
        message_count = 0
        last_message_count_time = time.time()
        
        try:
            while self.running:
                try:
                    # Read data if available
                    if serial_port.in_waiting:
                        data = serial_port.read(serial_port.in_waiting)
                        try:
                            text = data.decode('ascii', errors='replace')
                            message_buffer += text
                        except Exception as e:
                            logger.debug(f"Decode error: {e}")
                            message_buffer = ""
                            continue
                        
                        # Process complete NMEA sentences
                        while '\n' in message_buffer:
                            line_end = message_buffer.find('\n')
                            sentence = message_buffer[:line_end].strip()
                            message_buffer = message_buffer[line_end+1:]
                            
                            if sentence and sentence.startswith('$'):
                                message_count += 1
                                self._parse_nmea(gps_name, sentence)
                    
                    # Update message rate calculation every second
                    now = time.time()
                    if now - last_message_count_time >= 1.0:
                        elapsed = now - last_message_count_time
                        self.gps_data[gps_name]["messages_per_sec"] = message_count / elapsed
                        message_count = 0
                        last_message_count_time = now
                    
                    # Small delay to prevent CPU hogging
                    time.sleep(0.01)
                    
                except Exception as e:
                    logger.error(f"Error in {gps_name} GPS reader: {e}")
                    time.sleep(1)  # Avoid rapid error loops
                    
        except Exception as e:
            logger.error(f"Fatal error in {gps_name} GPS reader thread: {e}")
    
    def _parse_nmea(self, gps_name, sentence):
        """Parse NMEA sentences and update GPS data"""
        try:
            parts = sentence.split(',')
            if len(parts) < 2:
                return
                
            sentence_type = parts[0]
            
            # GGA - Time, position, and fix data
            if sentence_type in ("$GPGGA", "$GNGGA"):
                # Check if we have enough parts
                if len(parts) < 15:
                    return
                    
                # Parse fix quality
                try:
                    fix_quality = int(parts[6]) if parts[6] else 0
                    self.gps_data[gps_name]["fix_quality"] = fix_quality
                    
                    # Parse satellites in use
                    if parts[7]:
                        self.gps_data[gps_name]["satellites"] = int(parts[7])
                    
                    # Parse HDOP
                    if parts[8]:
                        self.gps_data[gps_name]["hdop"] = float(parts[8])
                    
                    # Only update position if we have a fix
                    if fix_quality > 0:
                        # Extract latitude
                        if parts[2] and parts[3]:
                            lat_deg = float(parts[2][:2])
                            lat_min = float(parts[2][2:])
                            lat_decimal = lat_deg + (lat_min / 60.0)
                            if parts[3] == 'S':
                                lat_decimal = -lat_decimal
                            self.gps_data[gps_name]["lat"] = lat_decimal
                        
                        # Extract longitude
                        if parts[4] and parts[5]:
                            lon_deg = float(parts[4][:3])
                            lon_min = float(parts[4][3:])
                            lon_decimal = lon_deg + (lon_min / 60.0)
                            if parts[5] == 'W':
                                lon_decimal = -lon_decimal
                            self.gps_data[gps_name]["lon"] = lon_decimal
                        
                        self.gps_data[gps_name]["last_update"] = time.time()
                except Exception as e:
                    logger.debug(f"Error parsing GGA: {e}")
            
            # RMC - Recommended minimum data
            elif sentence_type in ("$GPRMC", "$GNRMC"):
                try:
                    # Check if data is valid
                    if parts[2] == 'A':
                        # Extract speed in knots, convert to m/s
                        if parts[7]:
                            speed_knots = float(parts[7])
                            speed_ms = speed_knots * 0.514444
                            self.gps_data[gps_name]["speed"] = speed_ms
                        
                        # Extract heading/course
                        if parts[8]:
                            heading = float(parts[8])
                            self.gps_data[gps_name]["heading"] = heading
                except Exception as e:
                    logger.debug(f"Error parsing RMC: {e}")
                    
            # VTG - Course and speed information
            elif sentence_type in ("$GPVTG", "$GNVTG"):
                try:
                    # Extract true heading
                    if parts[1]:
                        heading = float(parts[1])
                        self.gps_data[gps_name]["heading"] = heading
                    
                    # Extract speed in km/h, convert to m/s
                    if parts[7]:
                        speed_kmh = float(parts[7])
                        speed_ms = speed_kmh / 3.6
                        self.gps_data[gps_name]["speed"] = speed_ms
                except Exception as e:
                    logger.debug(f"Error parsing VTG: {e}")
            
        except Exception as e:
            logger.debug(f"Error parsing NMEA sentence: {e}")
    
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
                        # Use geopy's geodesic distance for accurate movement simulation
                        origin = (front_lat, front_lon)
                        
                        # Calculate destination based on bearing and distance
                        d = distance / 1000.0  # Convert to kilometers for this calculation
                        R = 6371.0  # Earth's radius in km
                        
                        lat1 = math.radians(front_lat)
                        lon1 = math.radians(front_lon)
                        
                        # Calculate new position (accurately)
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
                    # For increased accuracy, calculate using the geodesic approach
                    rear_heading_rad = math.radians((sim_heading + 180) % 360)
                    rear_distance = 0.3 / 1000.0  # 30cm in kilometers
                    
                    # Front is our origin now
                    lat1 = math.radians(self.gps_data["front"]["lat"])
                    lon1 = math.radians(self.gps_data["front"]["lon"])
                    
                    # Calculate rear position
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
        
        # For real hardware mode, use the GPS data we've collected
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        lat, lon = None, None
        heading = None
        speed = 0.0
        
        # First, try to get position from front GPS
        if front["lat"] is not None and front["lon"] is not None:
            lat = front["lat"]
            lon = front["lon"]
            speed = front["speed"]
            
            # Try to get heading from the dual-GPS setup
            if rear["lat"] is not None and rear["lon"] is not None:
                # Calculate heading from dual GPS positions
                heading = self.calculate_bearing(
                    rear["lat"], rear["lon"],
                    front["lat"], front["lon"]
                )
                logger.debug("Heading calculated from dual GPS positions")
            
            # If dual-GPS heading failed, try using NMEA heading data
            if heading is None and front["heading"] is not None:
                heading = front["heading"]
                logger.debug("Using NMEA heading from front GPS")
        
        # If still no heading and we have multiple positions in history, try calculating from movement
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
        
        # Calculate heading between GPSes if possible
        calculated_heading = None
        if None not in (rear["lat"], rear["lon"], front["lat"], front["lon"]):
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
        
        if not self.simulation_mode:
            if hasattr(self, 'front_serial'):
                self.front_serial.close()
            if hasattr(self, 'rear_serial'):
                self.rear_serial.close()
    
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
        
        # Calculate bearing from rear to front GPS
        calculated_bearing = self.calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
        
        # Get the heading from GPS module
        reported_heading = self.gps_data["front"]["heading"]
        
        # Print debug info
        logger.info(f"===== BEARING DEBUG =====")
        logger.info(f"Front GPS: ({front_lat:.7f}, {front_lon:.7f})")
        logger.info(f"Rear GPS: ({rear_lat:.7f}, {rear_lon:.7f})")
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
            
        logger.info(f"========================")
        
        return calculated_bearing
    
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
    
    # Test bearing calculation
    print("Testing bearing calculation...")
    
    # Test point pairs (real-world examples)
    test_points = [
        # From->To coordinates, Expected bearing
        ((34.1519056, -77.8667716), (34.1519829, -77.8669909), 295.3),
        ((34.2257, -77.9447), (34.2257, -77.9445), 90.0),  # Due East
        ((34.2257, -77.9447), (34.2257, -77.9449), 270.0), # Due West
        ((34.2257, -77.9447), (34.2259, -77.9447), 0.0),   # Due North
        ((34.2257, -77.9447), (34.2255, -77.9447), 180.0)  # Due South
    ]
    
    monitor = GPSMonitor(simulation_mode=True)
    
    for i, ((lat1, lon1), (lat2, lon2), expected) in enumerate(test_points):
        bearing = monitor.calculate_bearing(lat1, lon1, lat2, lon2)
        diff = min(abs(bearing - expected), abs(bearing - expected + 360), abs(bearing - expected - 360))
        
        print(f"Test {i+1}: ({lat1}, {lon1}) to ({lat2}, {lon2})")
        print(f"  Calculated: {bearing:.1f}°, Expected: {expected:.1f}°, Diff: {diff:.1f}°")
        print(f"  {'PASS' if diff < 0.5 else 'FAIL'}")
    
    # Test distance calculation
    print("\nTesting distance calculation...")
    test_pos = (34.1519056, -77.8667716)
    test_waypoints = [
        (34.1519056, -77.8670785),  # WP 0
        (34.1521149, -77.8670045),  # WP 1
        (34.1521348, -77.8667011),  # WP 2
        (34.1519554, -77.8666744)   # WP 3
    ]
    
    for i, waypoint in enumerate(test_waypoints):
        dist, bearing = monitor.debug_distance_calculation(test_pos[0], test_pos[1], waypoint)
        print(f"WP {i}: Distance = {dist:.1f}m, Bearing = {bearing:.1f}°")


if __name__ == "__main__":
    test_gps_monitor()
