import numpy as np
import json
from geopy.distance import geodesic
from hardware.gps_monitor import GPSMonitor
from hardware.motor_controller import MotorController

class StanleyController:
    def __init__(self, waypoints, k=4.8, k_e=10.0, L=1.0, max_steer=np.radians(50), waypoint_reach_threshold=1.0):
        self.waypoints = waypoints
        self.k = k
        self.k_e = k_e
        self.L = L
        self.max_steer = max_steer
        self.waypoint_reach_threshold = waypoint_reach_threshold  # Add this line
        self.target_idx = 1  # Initial target waypoint index
        self.gps_monitor = GPSMonitor()
        self.motor_controller = MotorController()

    def add_extension_waypoint(self, extension_distance=1.0):
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
        
        return extended_waypoints

    def haversine_distance(self, coord1, coord2):
        return geodesic(coord1, coord2).meters

    def control_loop(self):
        while True:
            current_position = self.gps_monitor.get_position()
            x, y, yaw, v = current_position
            
            delta, self.target_idx, distance, cte, yaw_error = self.stanley_control(x, y, yaw, v)
            self.motor_controller.set_speed(v)
            self.motor_controller.set_steering(delta)

    def stanley_control(self, x, y, yaw, v):
        """
        Stanley controller for path tracking with proper CTE
        
        Args:
            x: Current x position (latitude)
            y: Current y position (longitude)
            yaw: Current heading angle (radians)
            v: Current velocity (m/s)
        """
        # Find nearest point on the path
        current_target = self.waypoints[self.target_idx]
        
        # Calculate distance to target
        distance = self.haversine_distance((x, y), (current_target[0], current_target[1]))
        
        # Check if we've reached the waypoint
        if distance < self.waypoint_reach_threshold and self.target_idx < len(self.waypoints) - 1:
            self.target_idx += 1
            return self.stanley_control(x, y, yaw, v)  # Recalculate with new target
        
        # Get current and target points for vector calculations
        prev_idx = max(0, self.target_idx - 1)
        prev_point = self.waypoints[prev_idx]
        current_point = self.waypoints[self.target_idx]
        
        # ===== CALCULATE CROSS-TRACK ERROR (CTE) =====
        # 1. Calculate path segment vector (from prev to current waypoint)
        path_vector = [current_point[0] - prev_point[0], current_point[1] - prev_point[1]]
        
        # 2. Calculate vector from prev point to current position
        pos_vector = [x - prev_point[0], y - prev_point[1]]
        
        # 3. Calculate path length
        path_length = np.sqrt(path_vector[0]**2 + path_vector[1]**2)
        
        # 4. Calculate projection of position onto path and CTE
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
        
        # ===== CALCULATE HEADING ERROR =====
        # Get current heading in degrees (0=North, 90=East)
        current_heading_deg = np.degrees(yaw) % 360
        
        # Calculate desired heading to target
        desired_heading_deg = self._calculate_bearing(x, y, current_point[0], current_point[1])
        
        # Calculate heading error
        heading_diff_deg = desired_heading_deg - current_heading_deg
        
        # Normalize to -180 to 180 range
        while heading_diff_deg > 180:
            heading_diff_deg -= 360
        while heading_diff_deg < -180:
            heading_diff_deg += 360
        
        # Convert to radians
        heading_diff_rad = np.radians(heading_diff_deg)
        
        # ===== STANLEY CONTROL LAW =====
        # 1. Heading error term
        heading_term = heading_diff_rad
        
        # 2. Cross-track error term
        # Ensure velocity is not zero to prevent division by zero
        v_safe = max(v, 0.1)
        cte_term = np.arctan2(self.k_e * cte, v_safe)
        
        # Combine terms for final steering command with proper gains
        delta = self.k * heading_term + cte_term
        
        # Limit steering angle
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        return delta, self.target_idx, distance, cte, heading_diff_rad

    def _haversine_distance(self, coord1, coord2):
        """Calculate the great circle distance between two points in meters"""
        from geopy.distance import geodesic
        return geodesic(coord1, coord2).meters

    def _normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def _calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points in degrees (0-360)"""
        import math
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        return (math.degrees(bearing) + 360) % 360

def main():
    with open('data/polygon_data.json') as f:
        polygon_data = json.load(f)
    
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    controller = StanleyController(waypoints)
    
    controller.control_loop()

if __name__ == "__main__":
    main()