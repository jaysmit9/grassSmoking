import argparse
import numpy as np
import json
import time
import sys
import os
from datetime import datetime
import logging
from geopy.distance import geodesic  # Add this import for geodesic calculations

# Import Stanley controller components
from controllers.stanley_controller import StanleyController

# Import hardware components
from hardware.gps_monitor import GPSMonitor  # Add this import
from hardware.gps_monitor import initialize_gps_with_retry
from hardware.motor_controller import get_motor_controller  # Add this import

# Conditionally import visualization modules for simulation mode
try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# Add this near the top of the file, after imports
def setup_logging():
    """Configure logging to both console and file"""
    import logging
    from datetime import datetime
    import os
    
    # Create a logs directory if it doesn't exist
    logs_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs')
    os.makedirs(logs_dir, exist_ok=True)
    
    # Create a timestamp for log files
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(logs_dir, f'rover_{timestamp}.log')
    
    # Setup root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    
    # Remove any existing handlers
    for handler in list(root_logger.handlers):
        root_logger.removeHandler(handler)
    
    # Add console handler for human-readable logs
    console_handler = logging.StreamHandler()
    console_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    console_handler.setFormatter(console_format)
    root_logger.addHandler(console_handler)
    
    # Add file handler for structured parseable logs
    file_handler = logging.FileHandler(log_file)
    file_format = logging.Formatter('%(asctime)s,%(levelname)s,%(message)s')
    file_handler.setFormatter(file_format)
    root_logger.addHandler(file_handler)
    
    # Create a special logger just for structured data
    data_logger = logging.getLogger('rover.data')
    data_logger.setLevel(logging.INFO)
    data_logger.propagate = False  # Don't send messages to root logger
    
    data_file = os.path.join(logs_dir, f'rover_data_{timestamp}.csv')
    data_handler = logging.FileHandler(data_file)
    data_format = logging.Formatter('%(asctime)s,%(message)s')
    data_handler.setFormatter(data_format)
    data_logger.addHandler(data_handler)
    
    print(f"Logs will be written to: {log_file}")
    print(f"Data will be written to: {data_file}")
    
    return root_logger, data_logger

# Call this at the beginning of your main function
logger, data_logger = setup_logging()

def load_waypoints(waypoints_file):
    """Load waypoints from file with better validation."""
    try:
        logger.info(f"Loading waypoints from: {waypoints_file}")
        waypoints = []
        
        # Determine file type and load accordingly
        if waypoints_file.endswith('.json'):
            with open(waypoints_file, 'r') as f:
                data = json.load(f)
                if isinstance(data, list) and len(data) > 0:
                    # Extract waypoints based on format
                    if isinstance(data[0], list):  # Simple format [[lat, lon], ...]
                        waypoints = np.array(data, dtype=np.float64)
                    elif isinstance(data[0], dict) and 'lat' in data[0] and 'lon' in data[0]:
                        waypoints = np.array([[point['lat'], point['lon']] for point in data], dtype=np.float64)
        else:
            # Assume it's a CSV or similar text format
            with open(waypoints_file, 'r') as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) >= 2:
                        try:
                            lat = float(parts[0])
                            lon = float(parts[1])
                            waypoints.append([lat, lon])
                        except ValueError:
                            logger.warning(f"Skipping invalid line: {line.strip()}")
                            continue
            waypoints = np.array(waypoints, dtype=np.float64)
        
        # Validate waypoints
        if len(waypoints) < 2:
            logger.error(f"Not enough valid waypoints found (need at least 2, found {len(waypoints)})")
            return np.array([[0, 0], [0.0001, 0.0001]])  # Provide dummy waypoints
        
        # Log all waypoints after loading (IMPORTANT)
        logger.info(f"Successfully loaded {len(waypoints)} waypoints:")
        for i, wp in enumerate(waypoints):
            logger.info(f"WP {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
            
        return waypoints
        
    except Exception as e:
        logger.error(f"Error loading waypoints: {e}")
        # Return dummy waypoints if loading fails
        return np.array([[0, 0], [0.0001, 0.0001]])

def run_simulation(waypoints_file, config=None):
    """Run the rover in simulation mode"""
    if not MATPLOTLIB_AVAILABLE:
        logger.error("Matplotlib is required for simulation mode")
        return
    
    # Default configuration updated for tanh-based controller
    if config is None:
        config = {
            'dt': 0.1,                      # Time step (seconds)
            'max_steer': np.radians(90),    # Maximum steering angle
            'target_speed': 2.0,            # Target speed (m/s)
            'extension_distance': 1.0,      # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0, # Distance threshold to consider waypoint reached
            'steering_sensitivity': np.pi/36  # Denominator for tanh function (lower = more aggressive)
        }
    
    # Load waypoints
    waypoints = load_waypoints(waypoints_file)
    
    # Initialize the controller with new parameters
    controller = StanleyController(
        waypoints=waypoints,
        max_steer=config['max_steer'],
        waypoint_reach_threshold=config['waypoint_reach_threshold'],
        steering_sensitivity=config.get('steering_sensitivity', np.pi/3)
    )
    
    # Add extension waypoint
    waypoints = controller.add_extension_waypoint(config['extension_distance'])
    
    # Rest of the function remains the same
    # Initial state setup
    x = waypoints[0, 0]  # Initial latitude
    y = waypoints[0, 1]  # Initial longitude
    v = 0.0              # Initial velocity
    
    # Set initial yaw to face the first waypoint
    dx = waypoints[1, 0] - x
    dy = waypoints[1, 1] - y
    yaw = np.arctan2(dy, dx)
    
    # Constants for geographic coordinate conversion
    METERS_PER_LAT_DEGREE = 111000
    
    # Run simulation
    logger.info("Starting simulation...")
    
    # Import simulation specific functions
    from simulation.simulator import simulate, plot_results, create_animation
    
    results = simulate(
        waypoints, x, y, yaw, v, 
        target_idx=1,  # Start with second waypoint as target
        target_speed=config['target_speed'],
        dt=config['dt'],
        max_steer=config['max_steer'],
        steering_sensitivity=config['steering_sensitivity'],  # New parameter
        waypoint_reach_threshold=config['waypoint_reach_threshold'],
        METERS_PER_LAT_DEGREE=METERS_PER_LAT_DEGREE
    )
    
    # Check if we should replay the simulation with progress logs
    replay_simulation = '--replay' in sys.argv
    
    # Write a header for the simulation log file
    data_logger.info("lat,lon,heading,speed,target_idx,total_waypoints,distance,cte,steering_deg,yaw_error_deg")
    
    if replay_simulation:
        logger.info("Replaying simulation with progress logging...")
        replay_duration = min(30.0, len(results['time']) * config['dt'])  # Max 30 seconds of replay
        replay_interval = 0.1  # Update every 100ms
        
        # Calculate how many steps to skip to fit within replay_duration
        total_steps = len(results['time'])
        step_skip = max(1, int(total_steps * replay_interval / replay_duration))
        
        # Create a trajectory file for the simulation too
        trajectory_file = open(f"sim_trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv", "w")
        trajectory_file.write("timestamp,lat,lon,heading,speed,target_idx,distance,cte\n")
        
        last_progress_log_time = time.time()
        
        for i in range(0, total_steps, step_skip):
            # Get state at this step
            x = results['x'][i]
            y = results['y'][i]
            yaw = results['yaw'][i]
            v = results['v'][i]
            target_idx = results['target_idx'][i]
            delta = results['delta'][i]
            cte = results['cte'][i]
            
            # Get distance to current waypoint
            if target_idx < len(waypoints):
                target_wp = waypoints[target_idx]
                distance = geodesic((x, y), (target_wp[0], target_wp[1])).meters
            else:
                distance = 0.0
            
            # Convert yaw to heading (0-360 degrees)
            heading = (np.degrees(yaw) + 360) % 360
            
            # Convert steering angle to degrees
            steering_degrees = np.degrees(delta)
            
            # Write to trajectory file
            current_time = time.time()
            trajectory_file.write(f"{current_time},{x},{y},{heading},"
                                 f"{v},{target_idx},{distance},{cte}\n")
            
            # Progress logging (once per second)
            if current_time - last_progress_log_time >= 1.0:
                last_progress_log_time = current_time
                
                # Calculate percentage to next waypoint if we know the previous waypoint
                progress_percent = "unknown"
                if target_idx > 0 and target_idx < len(waypoints):
                    # Calculate distance between previous and current waypoint
                    prev_wp = waypoints[target_idx - 1]
                    target_wp = waypoints[target_idx]
                    total_segment_dist = geodesic((prev_wp[0], prev_wp[1]), 
                                                (target_wp[0], target_wp[1])).meters
                    
                    # Calculate progress percentage
                    if total_segment_dist > 0:
                        dist_from_prev = geodesic((prev_wp[0], prev_wp[1]), 
                                                (x, y)).meters
                        progress = min(100, (dist_from_prev / total_segment_dist) * 100)
                        progress_percent = f"{progress:.1f}%"
                
                # Get bearing to target waypoint
                if target_idx < len(waypoints):
                    target_wp = waypoints[target_idx]
                    bearing_to_target = calculate_bearing(x, y, target_wp[0], target_wp[1])
                    
                    # Create progress bar
                    progress_bar_len = 20
                    if distance > 0:
                        threshold = config['waypoint_reach_threshold']
                        progress_normalized = min(1.0, max(0, (threshold - min(threshold, distance)) / threshold))
                        filled = int(progress_normalized * progress_bar_len)
                        progress_bar = f"[{'#' * filled}{'-' * (progress_bar_len - filled)}]"
                    else:
                        progress_bar = "[--------------------]"
                    
                    logger.info(f"\n===== SIMULATION PROGRESS =====")
                    logger.info(f"Waypoint: {target_idx}/{len(waypoints)-1} {progress_bar}")
                    logger.info(f"Position: {x:.6f}, {y:.6f} | Progress: {progress_percent}")
                    logger.info(f"Heading: {heading:.1f}° | Bearing to target: {bearing_to_target:.1f}°")
                    logger.info(f"Distance to waypoint: {distance:.2f}m | Cross-track error: {cte:.2f}m")
                    logger.info(f"Steering: {steering_degrees:.1f}° | Speed: {v:.2f} m/s")
                    logger.info(f"Time step: {i}/{total_steps} ({i/total_steps*100:.1f}%)")
                    logger.info(f"===============================")
            
            # Small delay to make the replay feel real-time
            time.sleep(replay_interval)
            
            # Log structured data for analysis
            structured_log = (
                f"{x:.7f},{y:.7f},"  # Position
                f"{np.degrees(yaw):.2f},{v:.2f},"  # Heading and speed
                f"{target_idx},{len(waypoints)-1},"  # Waypoint progress
                f"{distance:.2f},{cte:.2f},"  # Navigation metrics
                f"{np.degrees(delta)::.2f},{np.degrees(yaw_error):.2f}"  # Control values
            )
            data_logger.info(structured_log)
        
        # Close trajectory file
        trajectory_file.close()
        logger.info("Replay completed")
    
    # Plot results
    plot_results(results, waypoints)
    
    # Create animation if requested
    if '--animate' in sys.argv:
        create_animation(results, waypoints)
    
    logger.info("Simulation completed")

# Let's create a modified run_hardware function that uses the same patient approach as gps-test
def run_hardware(waypoints_file, config=None):
    """Run the rover with real hardware using the same GPS initialization pattern as gps-test"""
    # Default configuration
    if config is None:
        config = {
            'max_steer': np.radians(90),   # Maximum steering angle
            'speed': 0.30,                 # Target speed (m/s)
            'update_rate': 10,             # Control loop update rate (Hz)
            'extension_distance': 1.0,     # Extension distance beyond final waypoint
            'waypoint_reach_threshold': 1.0,  # Distance threshold to consider waypoint reached
            'steering_sensitivity': np.pi/3   # Denominator for tanh function
        }
    
    try:
        # Load waypoints from file
        waypoints = load_waypoints(waypoints_file)
        if len(waypoints) < 2:
            logger.error("Not enough waypoints to navigate. Need at least 2.")
            return
        
        # Initialize GPS - single instance, same pattern as gps-test
        logger.info("Initializing GPS...")
        gps = GPSMonitor()  # Direct instantiation, no retry function
        
        # Initialize motor controller after GPS
        motors = get_motor_controller(max_speed=config.get("speed", 0.3))
        logger.critical(f"MOTOR CONTROLLER MAX SPEED: {motors.max_speed}")
        
        # Initialize controller with shared GPS instance
        controller = StanleyController(
            waypoints=waypoints,
            max_steer=config.get("max_steer", np.radians(50)),
            waypoint_reach_threshold=config.get("waypoint_reach_threshold", 1.0),
            steering_sensitivity=config.get("steering_sensitivity", np.pi/3),
            gps_monitor=gps,  # Pass the same GPS instance!
            motor_controller=motors  # Pass the motors
        )
        
        # ======== GPS PREFLIGHT CHECK ========
        # Same patient approach as gps-test mode
        logger.info("Performing GPS preflight check...")
        gps_ready = False
        max_wait_time = 10  # seconds
        start_time = time.time()
        
        while not gps_ready and time.time() - start_time < max_wait_time:
            lat, lon, heading, speed = gps.get_position_and_heading()
            
            if lat is not None and lon is not None and heading is not None:
                logger.info(f"GPS data valid: ({lat:.7f}, {lon:.7f}), heading: {heading:.1f}°")
                gps_ready = True
            else:
                logger.warning("GPS position unavailable, waiting...")
                time.sleep(1)
        
        if not gps_ready:
            logger.error("GPS data not available after waiting. Check GPS installation.")
            return
        # ======================================
        
        # Start control loop
        logger.info("Starting hardware control loop. Press Ctrl+C to stop.")
        control_interval = 1.0 / config.get("update_rate", 10)
        
        while True:
            try:
                # Get current position and heading from GPS
                lat, lon, heading, speed = gps.get_position_and_heading()
                
                # Skip this iteration if we don't have valid GPS data
                if lat is None or lon is None or heading is None:
                    logger.warning("Waiting for valid GPS data... (lat=%s, lon=%s, heading=%s)", 
                                 lat, lon, heading)
                    time.sleep(1)  # Wait a bit longer than before
                    continue
                
                # Calculate the steering angle using the Stanley controller
                delta, target_idx, distance, _, _, _ = controller.stanley_control(
                    None, None, None, speed,
                    position=(lat, lon),
                    heading=heading
                )
                
                # Convert the steering angle to degrees for better readability
                steering_angle_deg = np.degrees(delta)
                
                # Apply the calculated steering angle to the robot
                motors.set_steering(steering_angle_deg)
                
                # Set the motor speed
                motors.set_speed(config["speed"])
                
                # Print debug information
                logger.info(f"POS: ({lat:.7f}, {lon:.7f}) | HDG: {heading:.1f}° | " + 
                           f"TGT: {target_idx} | DIST: {distance:.1f}m | STEER: {steering_angle_deg:.1f}°")
                
                # Wait before next iteration
                time.sleep(control_interval)
                
            except KeyboardInterrupt:
                logger.info("Control interrupted by user")
                motors.stop()
                break
            except Exception as e:
                logger.error(f"Error in control loop: {e}")
                logger.error(traceback.format_exc())
                motors.stop()
                time.sleep(1)  # Give a brief pause
        
    except Exception as e:
        logger.error(f"Error in run_hardware: {e}")
        logger.error(traceback.format_exc())

# Add this helper function
def debug_heading(gps, controller, waypoints):
    """Debug function to verify heading calculations"""
    lat, lon, heading, speed = gps.get_position_and_heading()
    
    if controller.target_idx < len(waypoints):
        target_idx = controller.target_idx
        target = waypoints[target_idx]
        
        # Calculate bearing using the consistent method
        # Use identical code to serial_gps_monitor.py
        import math
        lat1, lon1, lat2, lon2 = lat, lon, target[0], target[1]
        
        # Convert decimal degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2) 
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing
        dlon = lon2_rad - lon1_rad
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing_rad = math.atan2(x, y)
        
        # Convert to degrees and normalize to 0-360
        bearing = (math.degrees(bearing_rad) + 360) % 360
        
        # Rest of function remains the same...
        
        # Calculate desired steering direction
        heading_rad = np.radians(heading)
        bearing_rad = np.radians(bearing)
        
        # Normalize the difference to [-pi, pi]
        diff_rad = ((bearing_rad - heading_rad + np.pi) % (2 * np.pi)) - np.pi
        diff_deg = np.degrees(diff_rad)
        
        # Get distance to target
        dist = geodesic((lat, lon), (target[0], target[1])).meters
        
        # Print debugging info with prominent waypoint number
        logger.info(f"DEBUG - TARGET WP: {target_idx} [{dist:.1f}m] | "
                   f"GPS Heading: {heading:.1f}°, "
                   f"Bearing to target: {bearing:.1f}°, "
                   f"Needed turn: {diff_deg:.1f}° {'RIGHT' if diff_deg > 0 else 'LEFT'}")

# Add this function to your main.py

def run_gps_test(waypoints_file, config=None, test_position=None):
    """
    Test GPS readings and bearing calculations matching map visualizer.
    
    Args:
        waypoints_file: Path to waypoints file
        config: Configuration dictionary (unused)
        test_position: Optional tuple (lat, lon) for testing with a fixed position
    """
    try:
        import math
        from geopy.distance import geodesic
        import json
        import os
        
        # Fix file path resolution
        if not os.path.exists(waypoints_file):
            # Try alternate locations
            possible_paths = [
                waypoints_file,
                os.path.join('data', waypoints_file),
                os.path.join('..', 'data', waypoints_file),
                os.path.join('..', '..', 'data', waypoints_file),
                os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', waypoints_file),
                os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', waypoints_file)
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    waypoints_file = path
                    logger.info(f"Found waypoints file at: {path}")
                    break
                    
        if not os.path.exists(waypoints_file):
            logger.error(f"Could not find waypoints file: {waypoints_file}")
            return
        
        # Load waypoints exactly as the map visualizer does
        logger.info(f"Loading waypoints from {waypoints_file}")
        with open(waypoints_file, 'r') as f:
            waypoint_data = json.load(f)
        
        # Handle different JSON formats
        if isinstance(waypoint_data, list):
            # Direct array format
            if isinstance(waypoint_data[0], list):
                all_waypoints = waypoint_data
            # Object format with lat/lon
            elif isinstance(waypoint_data[0], dict) and 'lat' in waypoint_data[0]:
                all_waypoints = [[point['lat'], point['lon']] for point in waypoint_data]
            else:
                logger.error("Unknown waypoint format")
                return
        else:
            logger.error("Waypoint file doesn't contain an array")
            return
        
        # Ensure numpy array for consistent operation
        all_waypoints = np.array(all_waypoints)
        
        # Log loaded waypoints
        logger.info(f"Loaded {len(all_waypoints)} waypoints:")
        for i, wp in enumerate(all_waypoints):
            logger.info(f"WP {i}: ({wp[0]:.7f}, {wp[1]:.7f})")
        
        # Initialize GPS only if we need live data
        gps = None if test_position else GPSMonitor()
        
        # Constants
        update_interval = 1.0  # Update every 1 second
        last_update_time = 0
        
        # Calculate bearing EXACTLY as map_visualizer.py does
        def calculate_bearing(lat1, lon1, lat2, lon2):
            """Calculate bearing between two points exactly as in map_visualizer"""
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            dlon = lon2_rad - lon1_rad
            y = math.sin(dlon) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
            bearing = math.atan2(y, x)  # Note: atan2(y,x) NOT atan2(x,y)
            
            bearing_deg = math.degrees(bearing)
            bearing_deg = (bearing_deg + 360) % 360
            return bearing_deg
        
        # If test position is provided, just run once
        if test_position:
            lat, lon = test_position
            # Default heading (can be changed if needed)
            heading = 0.0
            
            # Print debug info about position source
            logger.info("⚠️ USING TEST POSITION (no live GPS)")
            logger.info(f"Position: ({lat:.7f}, {lon:.7f})")
            
            # Format output exactly like map_visualizer.py
            logger.info("\n=== WAYPOINT INFORMATION ===")
            logger.info(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
            logger.info("-" * 40)
            
            for i, wp in enumerate(all_waypoints):
                wp_lat, wp_lon = wp
                dist = geodesic((lat, lon), (wp_lat, wp_lon)).meters
                bearing = calculate_bearing(lat, lon, wp_lat, wp_lon)
                
                # Calculate turn direction if heading is available
                turn_info = ""
                if heading is not None:
                    relative_bearing = bearing - heading
                    relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
                    
                    if abs(relative_bearing) < 1:
                        turn_direction = "↑"
                    elif relative_bearing > 0:
                        turn_direction = "→"
                    else:
                        turn_direction = "←"
                        
                    turn_info = f"{turn_direction} {abs(relative_bearing):.1f}°"
                
                logger.info(f"{i:<4}{dist:<14.1f}{bearing:<10.1f}°{turn_info}")
            
            logger.info("=" * 40)
            
            # Print direct comparison with map visualizer test position
            map_test_pos = (34.1519056, -77.8667716)
            if lat != map_test_pos[0] or lon != map_test_pos[1]:
                logger.info("\n=== COMPARISON WITH MAP VISUALIZER TEST POSITION ===")
                logger.info(f"Map Visualizer position: {map_test_pos}")
                logger.info(f"Current test position: ({lat:.7f}, {lon:.7f})")
                
                # Calculate distance between positions
                pos_distance = geodesic(map_test_pos, (lat, lon)).meters
                logger.info(f"Distance between positions: {pos_distance:.2f}m")
                
                # Compare first waypoint distances
                if len(all_waypoints) > 0:
                    wp0 = all_waypoints[0]
                    map_dist = geodesic(map_test_pos, (wp0[0], wp0[1])).meters
                    test_dist = geodesic((lat, lon), (wp0[0], wp0[1])).meters
                    logger.info(f"WP0 from map position: {map_dist:.1f}m")
                    logger.info(f"WP0 from test position: {test_dist:.1f}m")
                    logger.info(f"Difference: {abs(map_dist - test_dist):.1f}m")
                
                logger.info("=" * 50)
            
            # Just return after printing once
            return
        
        # Live GPS mode - run continuously
        logger.info("Starting GPS test with live GPS data. Press Ctrl+C to exit.")
        
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
                
                # Print current position prominently
                logger.info(f"\n🌍 CURRENT POSITION: ({lat:.7f}, {lon:.7f})")
                if heading is not None:
                    logger.info(f"🧭 HEADING: {heading:.1f}°")
                
                # Format output exactly like map_visualizer.py
                logger.info("\n=== WAYPOINT INFORMATION ===")
                logger.info(f"{'WP':<4}{'Distance (m)':<14}{'Bearing':<10}{'Turn'}")
                logger.info("-" * 40)
                
                for i, wp in enumerate(all_waypoints):
                    wp_lat, wp_lon = wp
                    dist = geodesic((lat, lon), (wp_lat, wp_lon)).meters
                    bearing = calculate_bearing(lat, lon, wp_lat, wp_lon)
                    
                    # Calculate turn direction if heading is available
                    turn_info = ""
                    if heading is not None:
                        relative_bearing = bearing - heading
                        relative_bearing = ((relative_bearing + 180) % 360) - 180  # Normalize to -180 to 180
                        
                        if abs(relative_bearing) < 1:
                            turn_direction = "↑"
                        elif relative_bearing > 0:
                            turn_direction = "→"
                        else:
                            turn_direction = "←"
                            
                        turn_info = f"{turn_direction} {abs(relative_bearing):.1f}°"
                    
                    logger.info(f"{i:<4}{dist:<14.1f}{bearing:<10.1f}°{turn_info}")
                
                logger.info("=" * 40)
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("GPS test stopped by user")
    except Exception as e:
        logger.error(f"Error in GPS test: {e}")
        import traceback
        logger.error(traceback.format_exc())

# Add helper function for bearing calculation
def calculate_bearing(lat1, lon1, lat2, lon2):
    """Calculate bearing between two points in degrees (0-360)"""
    import math
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.atan2(x, y)
    
    return (math.degrees(bearing) + 360) % 360

def debug_bearing_algorithms(lat1, lon1, lat2, lon2):
    """Compare results from different bearing calculation algorithms"""
    # Method 1: Serial GPS Monitor formula
    def bearing_method1(lat1, lon1, lat2, lon2):
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Calculate bearing
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(x, y)
        
        # Convert to degrees and normalize to 0-360
        bearing_deg = (math.degrees(bearing) + 360) % 360
        return bearing_deg
    
    # Method 2: Geopy's formula
    from geopy.distance import geodesic
    def bearing_method2(lat1, lon1, lat2, lon2):
        # Use geopy's bearing calculation (returning 0-360)
        point1 = (lat1, lon1)
        point2 = (lat2, lon2)
        return geodesic(point1, point2).bearing
    
    # Method 3: Stanley Controller formula
    def bearing_method3(lat1, lon1, lat2, lon2):
        import math
        # Convert to radians
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
    
    # Calculate bearing using all methods
    b1 = bearing_method1(lat1, lon1, lat2, lon2)
    b2 = bearing_method2(lat1, lon1, lat2, lon2)
    b3 = bearing_method3(lat1, lon1, lat2, lon2)
    
    # Print debug results to ensure alignment
    logger.critical(f"BEARING COMPARISON:")
    logger.critical(f"  Points: ({lat1:.7f}, {lon1:.7f}) -> ({lat2:.7f}, {lon2:.7f})")
    logger.critical(f"  Method 1 (serial_gps_monitor): {b1:.1f}°")
    logger.critical(f"  Method 2 (geopy): {(b2 + 360) % 360:.1f}°")
    logger.critical(f"  Method 3 (Stanley): {b3:.1f}°")
    logger.critical(f"  MAX DIFFERENCE: {max(abs(b1-b2), abs(b1-b3), abs(b2-b3)):.2f}°")
    
    return b3  # Return the Stanley method for compatibility

# Move this function BEFORE run_hardware()
def verify_parameter_ordering(lat, lon, yaw, speed, waypoints, controller):
    """Verify correct parameter ordering between components"""
    # Get target waypoint
    target_idx = controller.target_idx
    if target_idx < len(waypoints):
        target_lat, target_lon = waypoints[target_idx]
        
        # Calculate bearings using different parameter orderings
        # 1. Serial monitor method (which we know works correctly)
        import math
        lat1, lon1, lat2, lon2 = lat, lon, target_lat, target_lon
        
        # Convert decimal degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Calculate bearing
        dlon = lon2_rad - lon1_rad
        x = math.sin(dlon) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing_rad = math.atan2(x, y)
        
        # Convert to degrees and normalize to 0-360
        serial_bearing = (math.degrees(bearing_rad) + 360) % 360
        
        # 2. Controller method
        controller_bearing = controller._calculate_bearing(lat, lon, target_lat, target_lon)
        
        # 3. Swapped current coords
        swapped_bearing = controller._calculate_bearing(lon, lat, target_lat, target_lon)
        
        # 4. Swapped target coords
        swapped_target_bearing = controller._calculate_bearing(lat, lon, target_lon, target_lat)
        
        # Log all for comparison
        logger.critical("===== BEARING ALGORITHM CHECK =====")
        logger.critical(f"Current: ({lat:.7f}, {lon:.7f}), Target: ({target_lat:.7f}, {target_lon:.7f})")
        logger.critical(f"Serial Monitor: {serial_bearing:.1f}°")
        logger.critical(f"Controller: {controller_bearing:.1f}°")
        logger.critical(f"Swapped Current: {swapped_bearing:.1f}°")
        logger.critical(f"Swapped Target: {swapped_target_bearing:.1f}°")
        logger.critical("================================")
        
        # Check parameter ordering
        if abs(serial_bearing - controller_bearing) > 0.1:
            logger.critical("⚠️ WARNING: BEARING CALCULATION MISMATCH!")
            
            # See if any swapped version matches
            if abs(serial_bearing - swapped_bearing) < 0.1:
                logger.critical("✅ SOLUTION: USE SWAPPED CURRENT COORDINATES")
            elif abs(serial_bearing - swapped_target_bearing) < 0.1:
                logger.critical("✅ SOLUTION: USE SWAPPED TARGET COORDINATES")

# Modify your main function to include the new GPS test mode
def main():
    """Main function to parse arguments and run the appropriate mode"""
    parser = argparse.ArgumentParser(description='Autonomous Rover Controller')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
    parser.add_argument('--real', action='store_true', help='Run with real hardware')
    parser.add_argument('--gps-test', action='store_true', help='Run in GPS test mode (no motors)')
    parser.add_argument('--test-position', help='Test position for GPS test mode as lat,lon', default=None)
    parser.add_argument('--waypoints', type=str, default='../data/polygon_data.json',
                        help='JSON file containing waypoints')
    parser.add_argument('--config', type=str, help='JSON file containing configuration')
    parser.add_argument('--animate', action='store_true', help='Create animation in simulation mode')
    parser.add_argument('--replay', action='store_true', help='Replay simulation with progress logging')
    parser.add_argument('--test-waypoints', action='store_true',
                        help='Use test waypoints matching GPS test output')
    
    args = parser.parse_args()
    
    # Parse test position if provided
    test_position = None
    if args.test_position:
        try:
            test_position = tuple(map(float, args.test_position.split(',')))
            logger.info(f"Using test position: {test_position}")
        except Exception as e:
            logger.error(f"Invalid test position format: {e}")
    
    # Load configuration if provided
    config = None
    if args.config:
        try:
            with open(args.config) as f:
                config = json.load(f)
        except Exception as e:
            logger.error(f"Failed to load configuration: {e}")
            sys.exit(1)
    
    # If test waypoints flag is set, override with hardcoded values from GPS test
    if args.test_waypoints:
        # These coordinates should be updated with what you see in --gps-test output
        test_waypoints = [
            (34.1519056, -77.8670785),  # WP 0 - matches 12.0m at bearing 277.8°
            (34.1521149, -77.8670045),  # WP 1 - approximated to match 18.8m, 339.2°
            (34.1521348, -77.8667011),  # WP 2 - approximated to match 13.1m, 14.4°
            (34.1519554, -77.8666744)   # WP 3 - approximated to match 2.1m, 53.3°
        ]
        waypoints = test_waypoints
        logger.info("Using hardcoded test waypoints that match --gps-test output")
        
        print("\n=== TEST WAYPOINTS LOADED ===")
        print("These coordinates should produce distances and bearings matching --gps-test")
    
    # Determine mode
    if args.sim and args.real:
        logger.error("Cannot specify both --sim and --real modes")
        sys.exit(1)
    elif args.sim:
        logger.info("Starting in SIMULATION mode")
        run_simulation(args.waypoints, config)
    elif args.real:
        logger.info("Starting in REAL HARDWARE mode")
        run_hardware(args.waypoints, config)
    elif args.gps_test:
        logger.info("Starting in GPS TEST mode")
        # Pass the test position
        run_gps_test(args.waypoints, config, test_position)
    else:
        logger.error("Must specify either --sim, --real, or --gps-test mode")
        parser.print_help()
        sys.exit(1)

if __name__ == "__main__":
    main()