def __init__(self, waypoints, k=6.0, k_e=1.0, L=1.0, max_steer=0.8726645886, waypoint_reach_threshold=1.0):
    self.waypoints = waypoints
    self.k = k  # Increased heading gain
    self.k_e = k_e  # Reduced CTE gain
    self.L = L
    self.max_steer = max_steer
    self.waypoint_reach_threshold = waypoint_reach_threshold
    self.target_idx = 1  # Initial target waypoint index
    self.gps_monitor = GPSMonitor()
    self.motor_controller = MotorController()
    # Initialize prev_delta to zero
    self.prev_delta = 0.0