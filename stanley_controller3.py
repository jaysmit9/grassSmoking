import numpy as np
import matplotlib.pyplot as plt
import json
from matplotlib.animation import FuncAnimation
from geopy.distance import geodesic

def main():
    # Load polygon data from JSON file
    with open('polygon_data.json') as f:
        polygon_data = json.load(f)

    # Extract coordinates from polygon data
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    
    # Initial state
    x = waypoints[0, 0]  # Initial latitude
    y = waypoints[0, 1]  # Initial longitude
    yaw = 0.0  # Initial heading (will be set based on first waypoint)
    v = 0.0    # Initial velocity
    
    # Set initial yaw to face the first waypoint
    dx = waypoints[1, 0] - x
    dy = waypoints[1, 1] - y
    yaw = np.arctan2(dy, dx)
    
    # Controller parameters - simpler values
    k = 0.8       # Stanley gain for heading error
    k_e = 1.0     # Stanley gain for cross-track error
    dt = 0.1      # Time step
    L = 0.5       # Wheelbase
    max_steer = np.radians(30)  # Maximum steering angle
    
    # Speed parameters
    target_speed = 2.0  # Target speed in m/s
    
    # Waypoint parameters
    target_idx = 1  # Initial target waypoint index
    
    # Distance threshold to consider waypoint reached
    waypoint_reach_threshold = 3.0  # meters
    
    # Constants for geographic coordinate conversion
    METERS_PER_LAT_DEGREE = 111000
    
    # Run simulation
    results = simulate(waypoints, x, y, yaw, v, target_idx, k, k_e, target_speed, 
                       dt, L, max_steer, waypoint_reach_threshold, METERS_PER_LAT_DEGREE)
    
    # Plot results
    plot_results(results, waypoints)
    
    # Create animation
    create_animation(results, waypoints)

def meters_per_lon_degree(lat):
    """Calculate meters per longitude degree at a given latitude"""
    METERS_PER_LAT_DEGREE = 111000
    return METERS_PER_LAT_DEGREE * np.cos(np.radians(lat))

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def calc_target_index(x, y, waypoints):
    """Find the closest waypoint to the vehicle"""
    distances = [haversine_distance((x, y), (point[0], point[1])) for point in waypoints]
    return np.argmin(distances)

def haversine_distance(coord1, coord2):
    """Calculate geographic distance between two coordinates"""
    return geodesic(coord1, coord2).meters

def stanley_control(x, y, yaw, v, waypoints, target_idx, k, k_e, waypoint_reach_threshold):
    """
    Stanley steering control
    
    Args:
        x: Current x position (latitude)
        y: Current y position (longitude)
        yaw: Current heading
        v: Current speed
        waypoints: Array of waypoints
        target_idx: Target waypoint index
        k: Control gain for heading error
        k_e: Control gain for cross-track error
        waypoint_reach_threshold: Distance threshold to consider waypoint reached
        
    Returns:
        delta: Steering angle
        target_idx: Target waypoint index
        distance_to_target: Distance to current target waypoint
        cte: Cross-track error
        yaw_error: Heading error
    """
    # Find the current and next waypoints
    current_waypoint = waypoints[max(0, target_idx-1)]
    target_waypoint = waypoints[target_idx]
    
    # Calculate desired heading to target
    dx = target_waypoint[0] - x
    dy = target_waypoint[1] - y
    desired_yaw = np.arctan2(dy, dx)
    
    # Calculate heading error
    yaw_error = normalize_angle(desired_yaw - yaw)
    
    # Calculate the cross-track error
    # We need to use vector projection on the path segment
    # First, the vector from previous waypoint to current waypoint
    wx = target_waypoint[0] - current_waypoint[0]
    wy = target_waypoint[1] - current_waypoint[1]
    
    # Vector from previous waypoint to current position
    vx = x - current_waypoint[0]
    vy = y - current_waypoint[1]
    
    # Project current position onto the path segment
    dot = wx * vx + wy * vy
    path_length_squared = wx*wx + wy*wy
    
    # Handle case of very short path segment
    if path_length_squared < 1e-10:
        cx = current_waypoint[0]
        cy = current_waypoint[1]
    else:
        # Normalized projection
        proj = dot / path_length_squared
        
        if proj < 0:  # Before the start of the segment
            cx = current_waypoint[0]
            cy = current_waypoint[1]
        elif proj > 1:  # After the end of the segment
            cx = target_waypoint[0]
            cy = target_waypoint[1]
        else:  # On the segment
            cx = current_waypoint[0] + proj * wx
            cy = current_waypoint[1] + proj * wy
    
    # Cross-track error - distance from current position to closest point on path
    cte = haversine_distance((x, y), (cx, cy))
    
    # Determine sign of cross-track error (left or right of path)
    vec1 = [wx, wy]
    vec2 = [x - current_waypoint[0], y - current_waypoint[1]]
    cross_z = vec1[0]*vec2[1] - vec1[1]*vec2[0]
    cte = cte * -np.sign(cross_z)  # Note the negative sign added
    
    # Calculate distance to target waypoint
    distance_to_target = haversine_distance((x, y), (target_waypoint[0], target_waypoint[1]))
    
    # Check if we've reached the target waypoint
    if target_idx < len(waypoints) - 1 and distance_to_target < waypoint_reach_threshold:
        target_idx += 1
        print(f"✓ Reached waypoint {target_idx-1}, moving to waypoint {target_idx}")
    
    # Calculate steering angle using Stanley controller formula
    # Stanley control law: heading_error + arctan(k_e * cte / (v + epsilon))
    epsilon = 0.1  # Avoid division by zero and increase steering at low speeds
    cte_term = np.arctan2(k_e * cte, v + epsilon)
    delta = yaw_error + cte_term
    
    # Print debugging info
    print(f"Distance to waypoint {target_idx}: {distance_to_target:.1f}m, "
          f"heading error: {np.degrees(yaw_error):.1f}°, CTE: {cte:.2f}m")
    
    return delta, target_idx, distance_to_target, cte, yaw_error

def simulate(waypoints, x, y, yaw, v, target_idx, k, k_e, target_speed, dt, L, max_steer,
            waypoint_reach_threshold, METERS_PER_LAT_DEGREE):
    """Simulate vehicle movement with Stanley controller"""
    # Track history
    x_history = [x]
    y_history = [y]
    yaw_history = [yaw]
    v_history = [v]
    target_history = [target_idx]
    cte_history = [0]
    reached_waypoints = [0]
    distance_history = [haversine_distance((x, y), (waypoints[target_idx][0], waypoints[target_idx][1]))]
    
    # Open debug file
    with open('debug_output.txt', 'w') as f:
        # Simulate until reaching final waypoint or max iterations
        max_iter = 1500
        for i in range(max_iter):
            if i % 50 == 0:
                print(f"Processing iteration {i}...")
                
            # Get steering input from Stanley controller
            delta, target_idx, distance, cte, heading_error = stanley_control(
                x, y, yaw, v, waypoints, target_idx, k, k_e, waypoint_reach_threshold
            )
                
            # Limit steering angle
            delta = np.clip(delta, -max_steer, max_steer)
            
            # Update velocity with simple proportional control
            if abs(delta) > np.radians(20):  # Slow down for sharp turns
                current_target_speed = target_speed * 0.6
            elif abs(delta) > np.radians(10):  # Moderate speed for moderate turns
                current_target_speed = target_speed * 0.8
            else:
                current_target_speed = target_speed
                
            dv = 0.5 * (current_target_speed - v)  # Simple proportional control
            v += dv * dt
            
            # Update position (convert to meters, then back to degrees)
            lat_change = v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
            lon_change = v * np.sin(yaw) * dt / meters_per_lon_degree(x)
            
            # Ensure minimum movement when velocity is significant
            if v > 0.3:
                min_movement = 0.0005
                min_lat = min_movement / METERS_PER_LAT_DEGREE
                min_lon = min_movement / meters_per_lon_degree(x)
                
                if abs(lat_change) < min_lat and abs(np.cos(yaw)) > 0.1:
                    lat_change = np.sign(np.cos(yaw)) * min_lat
                if abs(lon_change) < min_lon and abs(np.sin(yaw)) > 0.1:
                    lon_change = np.sign(np.sin(yaw)) * min_lon
            
            # Update position
            x += lat_change
            y += lon_change
            
            # Update heading
            # Calculate appropriate turning factor based on speed
            turning_factor = 1.0  # Default turning factor
            if v < 0.5:  # At low speeds, increase turning ability
                turning_factor = 3.0
            
            # Update yaw (heading) with enhanced turning at low speeds
            yaw += v * np.tan(delta) * turning_factor / L * dt
            yaw = normalize_angle(yaw)
            
            # Record history
            x_history.append(x)
            y_history.append(y)
            yaw_history.append(yaw)  # Store yaw, not y (fixed)
            v_history.append(v)
            target_history.append(target_idx)
            distance_history.append(distance)
            
            # Record reached waypoints
            if i > 0 and target_idx != target_history[-2]:
                reached_waypoints.append(target_idx - 1)
            
            # Write debug info
            f.write(f"Iteration {i}: x={x}, y={y}, yaw={yaw}, v={v}, steer_angle={delta}, "
                    f"target_idx={target_idx}, distance_to_target={distance}, "
                    f"cte={cte:.4f}, heading_error={np.degrees(heading_error):.2f}°\n")
            
            # Check if we've reached the final waypoint
            if target_idx == len(waypoints) - 1 and distance < waypoint_reach_threshold:
                print(f"✓✓ REACHED FINAL WAYPOINT at iteration {i}")
                reached_waypoints.append(target_idx)
                break
    
    # Return simulation results
    return {
        'x': x_history,
        'y': y_history,
        'yaw': yaw_history,
        'v': v_history,
        'target': target_history,
        'distance': distance_history,
        'reached_waypoints': reached_waypoints
    }

def plot_results(results, waypoints):
    """Plot simulation results"""
    plt.figure(figsize=(12, 10))
    
    # Plot waypoint path and vehicle path
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')
    plt.plot(results['x'], results['y'], 'b-', linewidth=2, label='Vehicle Path')
    
    # Mark start and end points
    plt.scatter(results['x'][0], results['y'][0], color='green', s=100, label='Start')
    plt.scatter(results['x'][-1], results['y'][-1], color='purple', s=100, label='End')
    
    # Mark reached waypoints
    for idx in results['reached_waypoints']:
        if idx < len(waypoints):
            plt.scatter(waypoints[idx, 0], waypoints[idx, 1], color='lime', s=150, marker='*')
    
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.title('Stanley Controller - GPS Waypoint Navigation')
    plt.grid(True)
    plt.legend()
    plt.savefig('stanley_path.png')
    plt.show()
    
    # Plot velocity profile
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(results['v'])), results['v'], 'g-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('Velocity (m/s)')
    plt.title('Velocity Profile')
    plt.grid(True)
    plt.savefig('velocity_profile.png')
    plt.show()
    
    # Plot distance to target
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(results['distance'])), results['distance'], 'r-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('Distance to Target (m)')
    plt.title('Distance to Target vs. Iteration')
    plt.grid(True)
    plt.savefig('distance_profile.png')
    plt.show()

def create_animation(results, waypoints):
    """Create animation of vehicle movement"""
    print("Creating animation...")
    fig, ax = plt.subplots(figsize=(12, 10))
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')
    
    # Path line and vehicle marker
    path_line, = plt.plot([], [], 'b-', linewidth=2, label='Vehicle Path')
    vehicle, = plt.plot([], [], 'bo', markersize=8)
    
    # Set axis limits with margin
    margin = 0.0001
    x_min = min(min(results['x']), min(waypoints[:, 0])) - margin
    x_max = max(max(results['x']), max(waypoints[:, 0])) + margin
    y_min = min(min(results['y']), min(waypoints[:, 1])) - margin
    y_max = max(max(results['y']), max(waypoints[:, 1])) + margin
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    
    # Skip frames for smoother animation
    skip = max(1, len(results['x']) // 200)
    frame_indices = list(range(0, len(results['x']), skip))
    if len(results['x'])-1 not in frame_indices:
        frame_indices.append(len(results['x'])-1)
    
    # Status text display
    status_text = plt.text(0.02, 0.95, '', transform=ax.transAxes,
                         bbox=dict(facecolor='white', alpha=0.7))
    
    # Vehicle heading indicator (arrow)
    arrow_length = 0.00005  # Arrow length in coordinates
    heading_arrow, = plt.plot([], [], 'r-', linewidth=2)
    
    def update(i):
        idx = frame_indices[i]
        path_line.set_data(results['x'][:idx+1], results['y'][:idx+1])
        vehicle.set_data([results['x'][idx]], [results['y'][idx]])
        
        # Update heading arrow
        arrow_x = [results['x'][idx], 
                  results['x'][idx] + arrow_length * np.cos(results['yaw'][idx])]
        arrow_y = [results['y'][idx], 
                  results['y'][idx] + arrow_length * np.sin(results['yaw'][idx])]
        heading_arrow.set_data(arrow_x, arrow_y)
        
        # Update status text
        target = results['target'][idx] if idx < len(results['target']) else results['target'][-1]
        status_text.set_text(
            f'Frame: {idx}/{len(results["x"])-1}\n'
            f'Target: {target}\n'
            f'Heading: {np.degrees(results["yaw"][idx]):.1f}°\n'
            f'Speed: {results["v"][idx]:.2f} m/s'
        )
        
        return path_line, vehicle, heading_arrow, status_text
    
    ani = FuncAnimation(fig, update, frames=len(frame_indices), interval=50, blit=True)
    
    plt.title('Stanley Controller Animation')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()