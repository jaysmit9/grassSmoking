def __init__(self, waypoints, k=5.0, k_e=3.0, L=1.0, max_steer=0.8726645886, waypoint_reach_threshold=1.0):
    # Rest of the initialization
    self.k = k  # Heading error gain
    self.k_e = k_e  # Cross-track error gain
    # ... rest of the code