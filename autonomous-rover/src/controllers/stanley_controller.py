import numpy as np
import json
import math
import time
import logging
import traceback
import requests  # Import requests for API calls
from requests.exceptions import RequestException  # For exception handling
from geopy.distance import geodesic
from hardware.gps_monitor_api import GPSMonitor
from hardware.motor_controller import MotorController

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Define the GPS API URL
GPS_API_BASE_URL = "http://localhost:5001/api"

class StanleyController:
    def __init__(self, waypoints, max_steer=np.radians(50), waypoint_reach_threshold=1.0, 
                 steering_sensitivity=np.pi/3, gps_monitor=None, motor_controller=None):
        """
        Initialize the Stanley Controller with simplified tanh-based steering.
        
        Args:
            waypoints: Array of waypoint coordinates (lat, lon)
            max_steer: Maximum steering angle in radians
            waypoint_reach_threshold: Distance in meters to consider a waypoint reached
            steering_sensitivity: Denominator for tanh function (lower = more aggressive)
            gps_monitor: Optional GPS monitor instance (can be None)
            motor_controller: Optional motor controller instance (can be None)
        """
        self.waypoints = waypoints
        self.max_steer = max_steer
        self.waypoint_reach_threshold = waypoint_reach_threshold
        self.steering_sensitivity = steering_sensitivity
        self.target_idx = 0
        self.curr_nearest_point_idx = 0
        
        # Store provided instances but don't create them
        self.gps_monitor = gps_monitor
        self.motor_controller = motor_controller
        
        self.first_call = True
        self.waypoint_0_visited = False
        
        # API session for connection pooling
        self.session = requests.Session()
        self.api_failures = 0
        self.max_api_failures = 5
        
        logger.info(f"Stanley Controller initialized with {len(waypoints)} waypoints")
        logger.info(f"Steering sensitivity: {np.degrees(steering_sensitivity):.1f}° (tanh denominator)")
        logger.info(f"Max steering angle: {np.degrees(max_steer):.1f}°")
        logger.info(f"Waypoint reach threshold: {waypoint_reach_threshold} meters")
        logger.info(f"Using GPS API at {GPS_API_BASE_URL}")
        logger.info(f"GPS monitor: {'Provided' if gps_monitor else 'None (using API)'}")
        logger.info(f"Motor controller: {'Provided' if motor_controller else 'None'}")

    def __del__(self):
        """Clean up resources"""
        if hasattr(self, 'session'):
            self.session.close()

    def add_extension_waypoint(self, extension_distance=1.0):
        """Add an extension waypoint beyond the final waypoint"""
        if len(self.waypoints) < 2:
            return self.waypoints
        
        final_waypoint = self.waypoints[-1]
        second_last_waypoint = self.waypoints[-2]
        
        # Use API to calculate bearing
        result = self.api_calculate_distance(
            second_last_waypoint[0], second_last_waypoint[1],
            final_waypoint[0], final_waypoint[1]
        )
        
        if result:
            bearing_rad = math.radians(result['bearing'])
            distance = result['distance']
        else:
            # Fall back to simple calculation
            lat_diff = final_waypoint[0] - second_last_waypoint[0]
            lon_diff = final_waypoint[1] - second_last_waypoint[1]
            
            bearing_rad = math.atan2(lon_diff, lat_diff)
            distance = self.haversine_distance(
                (second_last_waypoint[0], second_last_waypoint[1]),
                (final_waypoint[0], final_waypoint[1])
            )
        
        # Convert extension distance to degrees (approximate)
        METERS_PER_LAT_DEGREE = 111000
        lat_change_per_meter = 1.0 / METERS_PER_LAT_DEGREE
        lon_change_per_meter = 1.0 / (METERS_PER_LAT_DEGREE * np.cos(np.radians(final_waypoint[0])))
        
        # Calculate new waypoint coordinates
        extension_lat = final_waypoint[0] + extension_distance * lat_change_per_meter * math.cos(bearing_rad)
        extension_lon = final_waypoint[1] + extension_distance * lon_change_per_meter * math.sin(bearing_rad)
        
        extended_waypoints = np.copy(self.waypoints)
        extended_waypoints = np.vstack([extended_waypoints, [extension_lat, extension_lon]])
        
        logger.info(f"Added extension waypoint at ({extension_lat:.6f}, {extension_lon:.6f})")
        return extended_waypoints

    def haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
        try:
            # Extract coordinates
            lat1, lon1 = coord1
            lat2, lon2 = coord2
            
            # Try API first
            result = self.api_calculate_distance(lat1, lon1, lat2, lon2)
            if result:
                return result['distance']
            
            # Fall back to geopy
            return geodesic(coord1, coord2).meters
            
        except Exception as e:
            logger.error(f"Error in haversine_distance: {e}")
            return 0.0  # Safe default

    def api_calculate_distance(self, lat1, lon1, lat2, lon2):
        """Use the API to calculate distance and bearing"""
        if self.api_failures >= self.max_api_failures:
            return None  # Stop trying if we've had too many failures
            
        try:
            response = self.session.get(
                f"{GPS_API_BASE_URL}/gps/distance",
                params={
                    "lat1": lat1,
                    "lon1": lon1,
                    "lat2": lat2,
                    "lon2": lon2
                },
                timeout=0.5  # Short timeout for navigation needs
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('success'):
                    self.api_failures = 0  # Reset failure counter on success
                    return {
                        'distance': data['distance']['meters'],
                        'bearing': data['bearing']['degrees'],
                        'dx': data['cartesian']['dx'],
                        'dy': data['cartesian']['dy']
                    }
            
            # If we get here, API call failed
            self.api_failures += 1
            if self.api_failures == 1:  # Log only on first failure to avoid spam
                logger.warning(f"API distance calculation failed: {response.status_code}")
                
            return None
                
        except RequestException as e:
            self.api_failures += 1
            if self.api_failures <= 3:  # Limit log spam
                logger.warning(f"API request error: {e}")
            return None
            
        except Exception as e:
            logger.error(f"Unexpected error in API calculation: {e}")
            return None

    def get_current_position(self):
        """Get current position from GPS API"""
        try:
            response = self.session.get(
                f"{GPS_API_BASE_URL}/gps",
                timeout=1.0
            )
            
            if response.status_code == 200:
                data = response.json()
                
                # Extract front GPS position
                if "front_gps" in data:
                    lat = data["front_gps"].get("lat")
                    lon = data["front_gps"].get("lon")
                    
                    # Extract heading from dual GPS if available
                    heading = None
                    speed = 0.0
                    if "dual_gps" in data:
                        heading = data["dual_gps"].get("bearing")
                    
                    return lat, lon, heading, speed
            
            logger.warning(f"Failed to get position from API: {response.status_code}")
            return None, None, None, 0.0
                
        except Exception as e:
            logger.error(f"Error getting position from API: {e}")
            return None, None, None, 0.0

    def control_loop(self):
        """Main control loop for autonomous navigation using API data"""
        while True:
            try:
                # Get position from API instead of GPS monitor
                lat, lon, heading, speed = self.get_current_position()
                
                if lat is None or lon is None or heading is None:
                    logger.warning("Invalid position or heading from API")
                    time.sleep(0.5)  # Wait before retry
                    continue
                
                # Call stanley_control with the position data
                delta, self.target_idx, distance, cte, heading_error, _ = self.stanley_control(
                    lon, lat, np.radians(heading), speed
                )
                
                # Log important metrics
                logger.debug(f"Target waypoint: {self.target_idx}, Distance: {distance:.2f}m")
                logger.debug(f"Heading error: {np.degrees(heading_error):.1f}°, Steering: {np.degrees(delta):.1f}°")
                
                # Apply control if motor controller is available
                if self.motor_controller:
                    self.motor_controller.set_steering(np.degrees(delta))
                    self.motor_controller.set_speed(speed)
                
                # Short sleep to prevent CPU hogging
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                logger.info("Control loop stopped by user")
                if self.motor_controller:
                    self.motor_controller.set_speed(0)  # Stop motors
                break
                
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                if self.motor_controller:
                    self.motor_controller.set_speed(0)  # Stop motors for safety
                time.sleep(1)  # Wait before retry

    def stanley_control(self, x, y, yaw, speed, position=None, heading=None):
        """
        Simplified Stanley steering control using tanh function.
        
        IMPORTANT: The parameters are:
          x = longitude (not latitude)
          y = latitude (not longitude)
          yaw = heading in radians
          speed = velocity
          
        The position and heading parameters are optional and override x,y,yaw if provided
        """
        # If position and heading are provided, use those instead
        if position is not None:
            y, x = position  # position is (lat, lon)
        
        if heading is not None:
            yaw = np.radians(heading)
        
        # Check for None values in input parameters
        if x is None or y is None or yaw is None:
            logger.warning("stanley_control received None values: x=%s, y=%s, yaw=%s", x, y, yaw)
            # Return neutral steering and maintain current target
            return 0.0, self.target_idx, float('inf'), 0.0, 0.0, 0.0
        
        # Current target waypoint - waypoints are stored as [lat, lon]
        tx = self.waypoints[self.target_idx][0]  # lat
        ty = self.waypoints[self.target_idx][1]  # lon
        
        # Calculate distance to current target using API
        result = self.api_calculate_distance(y, x, tx, ty)
        
        if result:
            dist_to_target = result['distance']
            target_bearing = result['bearing']
        else:
            # Fall back to direct calculation
            current_point = (y, x)  # Note: y=lat, x=lon
            target_point = (tx, ty)
            dist_to_target = geodesic(current_point, target_point).meters
            target_bearing = self._calculate_bearing(y, x, tx, ty)
        
        # Current heading in degrees
        current_heading = np.degrees(yaw)
        
        # Calculate heading error
        heading_error_deg = target_bearing - current_heading
        
        # Normalize to -180 to +180
        heading_error_deg = ((heading_error_deg + 180) % 360) - 180
        
        # Debug info with bearing and heading
        logger.debug(f"Position: ({x:.6f}, {y:.6f}), Heading: {current_heading:.1f}°")
        logger.debug(f"Target: ({tx:.6f}, {ty:.6f}), Bearing: {target_bearing:.1f}°, Error: {heading_error_deg:.1f}°")
        
        # Convert back to radians for steering calculation
        heading_error = np.radians(heading_error_deg)
        
        # SIMPLIFIED STEERING WITH TANH
        steering_angle = np.tanh(heading_error / self.steering_sensitivity)
        
        # Scale to max_steer for steering control
        delta = steering_angle * self.max_steer
        
        logger.debug(f"Final steering angle: {np.degrees(delta):.1f}°")
        
        # Check if we've reached this waypoint
        if dist_to_target < self.waypoint_reach_threshold:
            # Mark waypoint 0 as visited if it's the current target
            if self.target_idx == 0:
                self.waypoint_0_visited = True
                
            # Move to next waypoint if available
            if self.target_idx < len(self.waypoints) - 1:
                self.target_idx += 1
                logger.info(f"Reached waypoint {self.target_idx-1}. New target: waypoint {self.target_idx}")
        
        # Return the steering angle, target index, distance, cross track error, heading error, and raw steering angle
        return delta, self.target_idx, dist_to_target, 0.0, heading_error, steering_angle

    def find_target_path_id(self, cx, cy, x, y):
        """
        Find the target index on the path.
        
        Args:
            cx: Path x coordinates array
            cy: Path y coordinates array
            x: Current x position
            y: Current y position
            
        Returns:
            int: Target index
        """
        # Calculate distance to all waypoints using API when possible
        distances = []
        for i, (lat, lon) in enumerate(zip(cy, cx)):  # cy=lat array, cx=lon array
            result = self.api_calculate_distance(y, x, lat, lon)
            if result:
                distances.append(result['distance'])
            else:
                # Fall back to direct calculation
                dx = x - lon
                dy = y - lat
                distances.append(math.sqrt(dx**2 + dy**2) * 111000)  # Approximate conversion to meters
        
        # Find closest waypoint
        target_idx = distances.index(min(distances))
        
        # If this is the first call, always start with waypoint 0
        if self.first_call:
            self.first_call = False
            return 0
        
        # Rest of the method remains the same...
        # Look at the two nearest waypoints
        if target_idx > 0 and target_idx < len(cx):
            # Get distances to the two nearest waypoints
            dist_to_current = distances[target_idx]
            dist_to_prev = distances[target_idx - 1]
            
            # IMPORTANT: Only move to the next waypoint if we're very close to the current one
            # or if we're already past it
            if self.target_idx < target_idx:
                # We're being asked to move forward in the path
                distance_threshold = 5.0  # metres
                if dist_to_current < distance_threshold:
                    # We're close enough to the current target, so use it
                    self.target_idx = target_idx
                else:
                    # Stay on the current target until we're close enough
                    target_idx = self.target_idx
            else:
                # We're being asked to move backward in the path - just use the closest
                self.target_idx = target_idx
        
        # If the logic above would make us skip waypoint 0, force it to be visited
        if self.target_idx > 0 and self.waypoint_0_visited == False:
            # Check if we're close enough to waypoint 0 to mark it as visited
            dist_to_wp0 = distances[0]
            if dist_to_wp0 < 3.0:  # 3 meter threshold
                self.waypoint_0_visited = True
            else:
                # Force targeting of waypoint 0
                self.target_idx = 0
                target_idx = 0
        
        return target_idx

    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate bearing between two points.
        Returns bearing in degrees (0-360).
        Uses API first, falls back to direct calculation.
        """
        # Try API first
        result = self.api_calculate_distance(lat1, lon1, lat2, lon2)
        if result:
            return result['bearing']
            
        # Fall back to direct calculation
        # Convert decimal degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing
        dlon = lon2_rad - lon1_rad
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = (math.degrees(bearing) + 360) % 360
        
        # Debug log
        logger.debug(f"BEARING CALC: ({lat1:.7f},{lon1:.7f}) → ({lat2:.7f},{lon2:.7f}) = {bearing_deg:.1f}°")
        
        return bearing_deg

def run_gps_test(waypoints_file, config=None):
    """Test GPS readings and bearing calculations using the GPS API"""
    try:
        # Import JSON and traceback here to avoid circular imports
        import json
        import traceback
        import numpy as np
        from datetime import datetime
        
        # Load waypoints
        with open(waypoints_file, 'r') as f:
            data = json.load(f)
            if isinstance(data, list) and len(data) > 0:
                # Extract waypoints based on format
                if isinstance(data[0], list):  # Simple format [[lat, lon], ...]
                    all_waypoints = np.array(data, dtype=np.float64)
                elif isinstance(data[0], dict) and 'lat' in data[0] and 'lon' in data[0]:
                    all_waypoints = np.array([[point['lat'], point['lon']] for point in data], dtype=np.float64)
            elif 'waypoints' in data:
                all_waypoints = np.array(data['waypoints'], dtype=np.float64)
            else:
                all_waypoints = np.array([])
        
        logger.info(f"Loaded {len(all_waypoints)} waypoints from {waypoints_file}")
        
        # Set up session for API calls
        session = requests.Session()
        
        # Set up variables for tracking position
        initial_lat = None
        initial_lon = None
        last_lat = None
        last_lon = None
        positions = []
        total_distance = 0.0
        
        # Constants
        update_interval = 1.0  # Update every 1 second
        last_update_time = 0
        
        logger.info("Starting GPS API test. Press Ctrl+C to exit.")
        
        def api_calculate_distance(lat1, lon1, lat2, lon2):
            """Use the API to calculate distance and bearing"""
            try:
                response = session.get(
                    f"{GPS_API_BASE_URL}/gps/distance",
                    params={
                        "lat1": lat1,
                        "lon1": lon1,
                        "lat2": lat2,
                        "lon2": lon2
                    },
                    timeout=0.5
                )
                
                if response.status_code == 200:
                    data = response.json()
                    if data.get('success'):
                        return {
                            'distance': data['distance']['meters'],
                            'bearing': data['bearing']['degrees'],
                            'dx': data['cartesian']['dx'],
                            'dy': data['cartesian']['dy']
                        }
                
                # If API fails, use geodesic
                from geopy.distance import geodesic
                return {
                    'distance': geodesic((lat1, lon1), (lat2, lon2)).meters,
                    'bearing': None  # Will be calculated separately
                }
                    
            except Exception as e:
                logger.error(f"Error in API distance calculation: {e}")
                return None
        
        def get_current_position():
            """Get current position from GPS API"""
            try:
                response = session.get(
                    f"{GPS_API_BASE_URL}/gps",
                    timeout=1.0
                )
                
                if response.status_code == 200:
                    data = response.json()
                    
                    # Extract front GPS position
                    if "front_gps" in data:
                        lat = data["front_gps"].get("lat")
                        lon = data["front_gps"].get("lon")
                        
                        # Extract heading from dual GPS if available
                        heading = None
                        speed = 0.0
                        if "dual_gps" in data:
                            heading = data["dual_gps"].get("bearing")
                        
                        return lat, lon, heading, speed
                
                return None, None, None, 0.0
                    
            except Exception as e:
                logger.error(f"Error getting position from API: {e}")
                return None, None, None, 0.0
        
        while True:
            current_time = time.time()
            
            # Check if it's time to update
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time
                
                # Get GPS data
                lat, lon, heading, speed = get_current_position()
                
                if lat is None or lon is None:
                    logger.warning("GPS position unavailable")
                    continue
                
                # Store position for movement tracking (max 10 recent positions)
                positions.append((lat, lon, current_time))
                if len(positions) > 10:
                    positions.pop(0)
                
                # Set initial position if not set
                if initial_lat is None:
                    initial_lat = lat
                    initial_lon = lon
                    logger.info(f"Initial position: {lat:.6f}, {lon:.6f}")
                
                # Calculate distance from start (if we have initial position)
                if initial_lat is not None and initial_lon is not None:
                    result = api_calculate_distance(initial_lat, initial_lon, lat, lon)
                    if result:
                        from_start = result['distance']
                        logger.info(f"Distance from start: {from_start:.1f}m")
                
                # Calculate distance from last point (if we have a previous position)
                if last_lat is not None and last_lon is not None:
                    result = api_calculate_distance(last_lat, last_lon, lat, lon)
                    if result:
                        from_last = result['distance']
                        total_distance += from_last
                        logger.info(f"Distance from last point: {from_last:.1f}m")
                        logger.info(f"Total distance traveled: {total_distance:.1f}m")
                
                # Update last position
                last_lat, last_lon = lat, lon
                
                # Calculate heading to each waypoint
                # Only do this if we have a valid heading
                if heading is not None:
                    logger.info(f"Current heading: {heading:.1f}°")
                    logger.info("All Waypoints:")
                    for i, wp in enumerate(all_waypoints):
                        wp_lat, wp_lon = wp
                        result = api_calculate_distance(lat, lon, wp_lat, wp_lon)
                        
                        if result:
                            dist = result['distance']
                            bear = result['bearing']
                            
                            # Calculate heading difference
                            heading_diff = ((bear - heading + 180) % 360) - 180
                            
                            logger.info(f"WP {i}: {dist:.1f}m at {bear:.1f}° (error: {'+' if heading_diff > 0 else ''}{heading_diff:.1f}°)")
                else:
                    logger.warning("GPS heading unavailable - waiting for valid heading")
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("GPS test stopped by user")
    except Exception as e:
        logger.error(f"Error in GPS test: {e}")
        logger.error(traceback.format_exc())
    finally:
        if 'session' in locals():
            session.close()

def main():
    """Test function for the controller"""
    with open('data/polygon_data.json') as f:
        polygon_data = json.load(f)
    
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    controller = StanleyController(waypoints)
    
    controller.control_loop()

if __name__ == "__main__":
    main()