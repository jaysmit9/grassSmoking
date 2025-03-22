import numpy as np
import math
from geopy.distance import geodesic
<<<<<<< HEAD
import json  # For main function

class StanleyController:
    def __init__(self, waypoints, k=6.0, k_e=0.2, L=1.0, max_steer=0.8726645886, waypoint_reach_threshold=3.0):
        # Initialize controller parameters
        self.waypoints = waypoints
        self.k = k           # Heading gain (keep as is)
        self.k_e = k_e       # DRASTICALLY REDUCE CTE GAIN from 1.0 to 0.2
        self.L = L
        self.max_steer = max_steer
        self.waypoint_reach_threshold = waypoint_reach_threshold
        self.target_idx = 1
        self.prev_delta = 0.0
        
        # Debug info
        print(f"Stanley controller initialized with {len(waypoints)} waypoints")
        print(f"Waypoint reach threshold: {waypoint_reach_threshold} meters")
        print(f"Targeting initial waypoint {self.target_idx}")
        print(f"Controller gains: k={k} (heading), k_e={k_e} (CTE)")
=======
from hardware.gps_monitor import GPSMonitor
from hardware.motor_controller import MotorController
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

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
        self.steering_sensitivity = steering_sensitivity  # New parameter for tanh sensitivity
        self.target_idx = 0  # Initial target waypoint index
        self.curr_nearest_point_idx = 0
        
        # Store provided instances but don't create them
        self.gps_monitor = gps_monitor
        self.motor_controller = motor_controller
        
        self.first_call = True
        self.waypoint_0_visited = False
        
        logger.info(f"Stanley Controller initialized with {len(waypoints)} waypoints")
        logger.info(f"Steering sensitivity: {np.degrees(steering_sensitivity):.1f}° (tanh denominator)")
        logger.info(f"Max steering angle: {np.degrees(max_steer):.1f}°")
        logger.info(f"Waypoint reach threshold: {waypoint_reach_threshold} meters")
        logger.info(f"GPS monitor: {'Provided' if gps_monitor else 'None'}")
        logger.info(f"Motor controller: {'Provided' if motor_controller else 'None'}")
>>>>>>> gpsd-refactor

    def add_extension_waypoint(self, extension_distance=1.0):
        """Add an extension waypoint beyond the final waypoint"""
        if len(self.waypoints) < 2:
            return self.waypoints
        
        final_waypoint = self.waypoints[-1]
        second_last_waypoint = self.waypoints[-2]
        
        lat_diff = final_waypoint[0] - second_last_waypoint[0]
        lon_diff = final_waypoint[1] - second_last_waypoint[1]
        
        distance = self.haversine_distance(
            (second_last_waypoint[0], second_last_waypoint[1]),
            (final_waypoint[0], final_waypoint[1])
        )
        
        if distance > 0:
            lat_diff /= distance
            lon_diff /= distance
        else:
            lat_diff = 1.0
            lon_diff = 0.0
        
        METERS_PER_LAT_DEGREE = 111000
        lat_change_per_meter = 1.0 / METERS_PER_LAT_DEGREE
        lon_change_per_meter = 1.0 / (METERS_PER_LAT_DEGREE * np.cos(np.radians(final_waypoint[0])))
        
        extension_lat = final_waypoint[0] + lat_diff * extension_distance * lat_change_per_meter
        extension_lon = final_waypoint[1] + lon_diff * extension_distance * lon_change_per_meter
        
        extended_waypoints = np.copy(self.waypoints)
        extended_waypoints = np.vstack([extended_waypoints, [extension_lat, extension_lon]])
        
        logger.info(f"Added extension waypoint at ({extension_lat:.6f}, {extension_lon:.6f})")
        return extended_waypoints

    def haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
<<<<<<< HEAD
        lat1, lon1 = coord1
        lat2, lon2 = coord2
        return geodesic((lat1, lon1), (lat2, lon2)).meters

    def stanley_control(self, x, y, yaw, v):
        """Stanley controller with adaptive gains"""
        # Normalize yaw and convert to degrees (keep existing code)
        yaw = yaw % (2 * np.pi)
        current_heading_deg = np.degrees(yaw) % 360
        
        # Print position (keep existing code)
        print(f"\nCurrent position: ({x:.6f}, {y:.6f}), heading: {np.degrees(yaw):.1f}°, speed: {v:.1f}m/s")
        
        # Find target waypoint (keep existing code)
        old_target = self.target_idx
        self.target_idx = self._find_target_waypoint(x, y, self.target_idx)
        
        # Log waypoint changes (keep existing code)
        if old_target != self.target_idx:
            print(f"TARGET CHANGED from {old_target} to {self.target_idx}")
            # Print all waypoints
        
        # Get waypoints and calculate distance (keep existing code)
        current_point = self.waypoints[self.target_idx]
        prev_idx = max(0, self.target_idx - 1)
        prev_point = self.waypoints[prev_idx]
        
        # CRITICAL DEBUG: Print target waypoint and current position
        print(f"Target waypoint: {current_point[0]:.7f}, {current_point[1]:.7f}")
        print(f"Current position: {x:.7f}, {y:.7f}")
        
        distance = self.haversine_distance((x, y), current_point)
        
        # CRITICAL DEBUG: Print calculated distance
        print(f"Calculated distance: {distance:.2f} meters")
        
        # Calculate CTE (keep existing code)
        # ===== CALCULATE CTE (MISSING CODE) =====
        # 1. Calculate path segment vector (from prev to current waypoint)
        path_vector = [current_point[0] - prev_point[0], current_point[1] - prev_point[1]]
=======
        return geodesic(coord1, coord2).meters

    def control_loop(self):
        """Main control loop for autonomous navigation"""
        while True:
            current_position = self.gps_monitor.get_position()
            x, y, yaw, v = current_position
            
            delta, self.target_idx, distance, cte, yaw_error = self.stanley_control(x, y, yaw, v)
            
            # Log important metrics
            logger.debug(f"Target waypoint: {self.target_idx}, Distance: {distance:.2f}m")
            logger.debug(f"Heading error: {np.degrees(yaw_error):.1f}°, Steering: {np.degrees(delta):.1f}°")
            
            # Apply control
            self.motor_controller.set_steering(np.degrees(delta))  # Convert to degrees for motor controller
            self.motor_controller.set_speed(v)

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
>>>>>>> gpsd-refactor
        
        # Current target waypoint - waypoints are stored as [lat, lon]
        tx = self.waypoints[self.target_idx][0]  # lat
        ty = self.waypoints[self.target_idx][1]  # lon
        
<<<<<<< HEAD
        # 3. Calculate path length - this was missing!
        path_length = np.sqrt(path_vector[0]**2 + path_vector[1]**2)
        
        # 4. Calculate projection of position onto path - this was missing!
        projection = 0  # Default value
        cte = 0  # Default value
        
        if path_length > 0:
            # Normalize path vector
            path_unit = [path_vector[0]/path_length, path_vector[1]/path_length]
            
            # Project position vector onto path
            projection = pos_vector[0]*path_unit[0] + pos_vector[1]*path_unit[1]
            projection = min(max(0, projection), path_length)  # Clamp to segment
            
            # Calculate projection point coordinates
            proj_point = [
                prev_point[0] + projection*path_unit[0],
                prev_point[1] + projection*path_unit[1]
            ]
            
            # Calculate CTE distance in meters
            cte_distance = self.haversine_distance((x, y), proj_point)
            
            # Determine sign of CTE (positive = right of path, negative = left of path)
            cross_z = path_vector[0]*pos_vector[1] - path_vector[1]*pos_vector[0]
            cte = cte_distance * (1 if cross_z >= 0 else -1)
        else:
            # If path is a point, CTE is just distance to the point
            cte = self.haversine_distance((x, y), prev_point)
            projection = 0
        # =================================================
        
        # Get bearing to target and calculate heading error (keep existing code)
        bearing_deg = self._calculate_bearing(x, y, current_point[0], current_point[1])
        heading_error_deg = bearing_deg - current_heading_deg
        while heading_error_deg > 180:
            heading_error_deg -= 360
        while heading_error_deg < -180:
            heading_error_deg += 360
        heading_error = np.radians(heading_error_deg)
        
        # Calculate distance to target waypoint
        distance = self.haversine_distance((x, y), self.waypoints[self.target_idx])
        
        # Adaptive gains based on distance
        k_scale = min(1.0, distance / 10.0)  # Scale from 0-1 up to 10m
        k_e_scale = min(1.0, distance / 5.0)  # Scale from 0-1 up to 5m
        
        adaptive_k = self.k * k_scale
        adaptive_k_e = self.k_e * k_e_scale
        
        # Calculate heading term
        heading_term = adaptive_k * heading_error
        
        # Calculate CTE term
        cte = self._calculate_cross_track_error(x, y, yaw)
        cte_term = np.arctan2(adaptive_k_e * cte, v)
        
        # Combine terms
        delta = heading_term + cte_term
        
        # Apply steering limits
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        # Apply smoothing filter
        if hasattr(self, 'prev_delta'):
            delta = 0.7 * delta + 0.3 * self.prev_delta
        
        # Store for next iteration
        self.prev_delta = delta
        
        return delta, self.target_idx, distance, cte, heading_error
=======
        # Calculate distance to current target
        current_point = (y, x)  # Note: y=lat, x=lon
        target_point = (tx, ty)
        dist_to_target = geodesic(current_point, target_point).meters
        
        # Calculate target bearing
        if hasattr(self, 'gps_monitor') and self.gps_monitor is not None:
            # Use GPS monitor's calculate_bearing if available
            target_bearing = self.gps_monitor.calculate_bearing(y, x, tx, ty)  # y=lat, x=lon
        else:
            # Calculate bearing directly
            # Convert decimal degrees to radians
            lat1_rad = math.radians(y)
            lon1_rad = math.radians(x)
            lat2_rad = math.radians(tx)
            lon2_rad = math.radians(ty)
            
            # Calculate bearing
            dlon = lon2_rad - lon1_rad
            y_val = math.sin(dlon) * math.cos(lat2_rad)
            x_val = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
            bearing_rad = math.atan2(y_val, x_val)
            
            # Convert to degrees and normalize to 0-360
            target_bearing = (math.degrees(bearing_rad) + 360) % 360
        
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
>>>>>>> gpsd-refactor

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
        # Dist to all waypoints
        dx = [x - icx for icx in cx]
        dy = [y - icy for icy in cy]
        d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
        target_idx = d.index(min(d))
        
        # If this is the first call, always start with waypoint 0
        if self.first_call:
            self.first_call = False
            return 0
        
        # Look at the two nearest waypoints
        if target_idx > 0 and target_idx < len(cx):
            # Get distances to the two nearest waypoints
            dist_to_current = d[target_idx]
            dist_to_prev = d[target_idx - 1]
            
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
            dist_to_wp0 = math.sqrt(dx[0]**2 + dy[0]**2)
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
        Calculate bearing between two points, using the EXACT same formula as serial_gps_monitor.
        Returns bearing in degrees (0-360).
        """
        import math
        # Convert decimal degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing (EXACT same formula as serial_gps_monitor)
        dlon = lon2_rad - lon1_rad
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = (math.degrees(bearing) + 360) % 360
        
        # Debug log
        import logging
        logger = logging.getLogger("rover")
        logger.debug(f"BEARING CALC: ({lat1:.7f},{lon1:.7f}) → ({lat2:.7f},{lon2:.7f}) = {bearing_deg:.1f}°")
        
        return bearing_deg

<<<<<<< HEAD
    def _find_target_waypoint(self, x, y, current_idx):
        """Find the target waypoint to follow"""
        # If we're at the start, find the closest waypoint
        if current_idx == 0:
            min_distance = float('inf')
            closest_idx = -1
            
            for i in range(len(self.waypoints)):
                waypoint = self.waypoints[i]
                distance = self.haversine_distance((x, y), waypoint)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_idx = i
            
            return closest_idx
        
        # Otherwise, continue to the next waypoint
        else:
            # Get the coordinates of the current waypoint
            current_waypoint = self.waypoints[current_idx]
            
            # Calculate the distance to the current waypoint
            distance = self.haversine_distance((x, y), current_waypoint)
            
            # If we've reached the current target within threshold
            if distance < self.waypoint_reach_threshold:
                # Move to next waypoint if we have more
                if current_idx < len(self.waypoints) - 1:
                    next_idx = current_idx + 1
                    return next_idx
                else:
                    return current_idx
            
            return current_idx

    def _calculate_cross_track_error(self, x, y, yaw):
        """Calculate cross track error"""
        # Find the nearest point on the path
        closest_point, target_idx, distance = self._find_closest_waypoint(x, y)
        
        # Calculate the angle between the robot and the path
        angle = np.arctan2(y - closest_point[1], x - closest_point[0])
        
        # Calculate the cross track error
        cross_track_error = distance * np.sin(angle - yaw)
        
        return cross_track_error

    def _find_closest_waypoint(self, x, y):
        """Find the closest waypoint to the current position"""
        min_distance = float('inf')
        closest_idx = -1
        
        for i in range(len(self.waypoints)):
            waypoint = self.waypoints[i]
            distance = self.haversine_distance((x, y), waypoint)
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = i
        
        return self.waypoints[closest_idx], closest_idx, min_distance
=======
def run_gps_test(waypoints_file, config=None):
    """Test GPS readings and bearing calculations"""
    try:
        # Load waypoints
        all_waypoints = load_waypoints(waypoints_file)
        
        # Initialize GPS
        gps = GPSMonitor()
        
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
        
        logger.info("Starting GPS test. Press Ctrl+C to exit.")
        
        while True:
            current_time = time.time()
            
            # Check if it's time to update
            if current_time - last_update_time >= update_interval:
                last_update_time = current_time
                
                # Get GPS data
                lat, lon, heading, speed = gps.get_position_and_heading()
                
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
                    from_start = geodesic((initial_lat, initial_lon), (lat, lon)).meters
                    logger.info(f"Distance from start: {from_start:.1f}m")
                
                # Calculate distance from last point (if we have a previous position)
                if last_lat is not None and last_lon is not None:
                    from_last = geodesic((last_lat, last_lon), (lat, lon)).meters
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
                        dist = geodesic((lat, lon), (wp_lat, wp_lon)).meters
                        bear = _calculate_bearing(lat, lon, wp_lat, wp_lon)
                        
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
>>>>>>> gpsd-refactor

def main():
    """Test function for the controller"""
    with open('data/polygon_data.json') as f:
        polygon_data = json.load(f)
    
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    controller = StanleyController(waypoints)
    
    controller.control_loop()

if __name__ == "__main__":
    main()