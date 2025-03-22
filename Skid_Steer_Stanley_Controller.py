import numpy as np
import matplotlib.pyplot as plt
import json
from matplotlib.animation import FuncAnimation
from geopy.distance import geodesic

def add_extension_waypoint(waypoints, extension_distance=1.0):
    """
    Add a waypoint that extends beyond the final waypoint by the specified distance
    """
    if len(waypoints) < 2:
        return waypoints
    
    # Get the last two waypoints
    final_waypoint = waypoints[-1]
    second_last_waypoint = waypoints[-2]
    
    # Calculate the direction vector
    lat_diff = final_waypoint[0] - second_last_waypoint[0]
    lon_diff = final_waypoint[1] - second_last_waypoint[1]
    
    # Calculate the distance between these points
    distance = haversine_distance(
        (second_last_waypoint[0], second_last_waypoint[1]),
        (final_waypoint[0], final_waypoint[1])
    )
    
    # Normalize the direction vector
    if distance > 0:
        lat_diff = lat_diff / distance
        lon_diff = lon_diff / distance
    else:
        # If waypoints are identical, use North direction
        lat_diff = 1.0
        lon_diff = 0.0
    
    # Calculate the lat/lon changes needed for 1 meter
    # For latitude, 1 degree is approximately 111,000 meters
    METERS_PER_LAT_DEGREE = 111000
    lat_change_per_meter = 1.0 / METERS_PER_LAT_DEGREE
    
    # For longitude, it depends on the latitude (narrower at poles)
    lon_change_per_meter = 1.0 / (METERS_PER_LAT_DEGREE * np.cos(np.radians(final_waypoint[0])))
    
    # Create the extension waypoint (1 meter further in the same direction)
    extension_lat = final_waypoint[0] + lat_diff * extension_distance * lat_change_per_meter
    extension_lon = final_waypoint[1] + lon_diff * extension_distance * lon_change_per_meter
    
    # Add the new waypoint
    extended_waypoints = np.copy(waypoints)
    extended_waypoints = np.vstack([extended_waypoints, [extension_lat, extension_lon]])
    
    print(f"Added extension waypoint: ({extension_lat}, {extension_lon})")
    print(f"Distance from final waypoint: {haversine_distance((final_waypoint[0], final_waypoint[1]), (extension_lat, extension_lon)):.2f} meters")
    
    return extended_waypoints

def main():
    # Load polygon data from JSON file
    with open('polygon_data.json') as f:
        polygon_data = json.load(f)

    # Extract coordinates from polygon data
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    
    # Add an extension waypoint 1 meter beyond the final waypoint
    waypoints = add_extension_waypoint(waypoints, 1.0)
    
    # Initial state
    x = waypoints[0, 0]  # Initial latitude
    y = waypoints[0, 1]  # Initial longitude
    yaw = 0.0  # Initial heading (will be set based on first waypoint)
    v = 0.5  # Initial velocity (non-zero to get things started)
    
    # Set initial yaw to face the first waypoint
    dx = waypoints[1, 0] - x
    dy = waypoints[1, 1] - y
    yaw = np.arctan2(dy, dx)
    
    # Controller parameters
    k = 4.2       # Stanley gain for heading error
    k_e = 7.0    # Stanley gain for cross-track error
    dt = 0.1      # Time step
    B = 1.1       # Track width (distance between wheels)
    max_speed_diff = 1.0  # Maximum speed difference between wheels (m/s)

    # Speed parameters
    target_speed = 2.0  # Target speed in m/s
    
    # Waypoint parameters
    target_idx = 1  # Initial target waypoint index
    
    # Distance threshold to consider waypoint reached
    waypoint_reach_threshold = 1.0  # meters
    
    # Constants for geographic coordinate conversion
    METERS_PER_LAT_DEGREE = 111000
    
    # Run simulation
    results = simulate(waypoints, x, y, yaw, v, target_idx, k, k_e, target_speed, 
                       dt, B, max_speed_diff, waypoint_reach_threshold, METERS_PER_LAT_DEGREE)
    
    # Plot results - only show the first plot
    plot_results(results, waypoints, show_first_only=True)
    
    # Comment out the animation call
    # create_animation(results, waypoints)

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
    Stanley controller for path following - same as before, but now returns a desired
    steering command that we'll convert to wheel speeds
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
    # Vector from previous waypoint to current waypoint
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
    cte = cte * -np.sign(cross_z)  # Use negative sign for proper geographic interpretation
    
    # Calculate distance to target waypoint
    distance_to_target = haversine_distance((x, y), (target_waypoint[0], target_waypoint[1]))
    
    # Save original target_idx for return value
    original_target_idx = target_idx
    
    # Check if we've reached the target waypoint
    if target_idx < len(waypoints) - 1 and distance_to_target < waypoint_reach_threshold:
        target_idx += 1
        print(f"✓ Reached waypoint {target_idx-1}, moving to waypoint {target_idx}")
        
        # Hide extension waypoint from user display if it's the final waypoint
        if target_idx == len(waypoints) - 1:
            print(f"Moving to final waypoint")
    
    # Calculate steering angle using Stanley controller formula
    epsilon = 0.1  # Avoid division by zero and increase steering at low speeds
    cte_term = np.arctan2(k_e * cte, v + epsilon)
    delta = k * yaw_error + cte_term
    
    # Debugging info
    print(f"Distance to waypoint {original_target_idx}: {distance_to_target:.1f}m, "
          f"heading error: {np.degrees(yaw_error):.1f}°, CTE: {cte:.2f}m")
    
    return delta, target_idx, distance_to_target, cte, yaw_error

def simulate(waypoints, x, y, yaw, v, target_idx, k, k_e, target_speed, dt, B, max_speed_diff,
            waypoint_reach_threshold, METERS_PER_LAT_DEGREE):
    """Simulate vehicle movement with skid-steer control"""
    # Track history
    x_history = [x]
    y_history = [y]
    yaw_history = [yaw]
    v_history = [v]
    v_left_history = [0]
    v_right_history = [0]
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
                
            # Convert steering angle delta to wheel velocities for skid steering
            # Limit the steering angle first
            delta = np.clip(delta, -np.pi/2, np.pi/2)
            
            # Calculate desired angular velocity (yaw rate) from steering delta
            omega_desired = v * np.sin(delta) / (B/2)
            
            # Limit the omega to respect max_speed_diff
            omega = np.clip(omega_desired, -max_speed_diff/(B/2), max_speed_diff/(B/2))
            
            # Calculate left and right wheel speeds
            v_left = v - (B/2) * omega
            v_right = v + (B/2) * omega
            
            # Ensure minimum speeds and avoid negative velocity (only drive forward)
            v_left = max(0.1, v_left) if v > 0.1 else 0
            v_right = max(0.1, v_right) if v > 0.1 else 0
            
            # Update vehicle velocity (average of wheel speeds)
            v = (v_left + v_right) / 2
            
            # Apply speed limits
            if v > target_speed:
                scale = target_speed / v
                v_left *= scale
                v_right *= scale
                v = target_speed
            
            # Add speed control with proper acceleration
            accel_rate = 0.05  # Acceleration rate (m/s²)
            
            # Always accelerate toward target speed
            if v < target_speed:
                # Apply acceleration but limit to accel_rate * dt
                v += min(accel_rate * dt, target_speed - v)
                
                # Adjust left/right wheel speeds proportionally 
                if v_left + v_right > 0:  # Avoid division by zero
                    avg_speed = (v_left + v_right) / 2
                    v_left = v_left * (v / avg_speed) if avg_speed > 0 else v/2
                    v_right = v_right * (v / avg_speed) if avg_speed > 0 else v/2
                else:
                    v_left = v/2
                    v_right = v/2
            elif v > target_speed:
                # Existing deceleration code
                scale = target_speed / v
                v_left *= scale
                v_right *= scale
                v = target_speed
            
            # Update position (convert to meters, then back to degrees)
            lat_change = v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
            lon_change = v * np.sin(yaw) * dt / meters_per_lon_degree(x)
            
            # Update position
            x += lat_change
            y += lon_change
            
            # Update heading based on differential wheel speeds
            # For skid steering: omega = (v_right - v_left) / B
            omega_actual = (v_right - v_left) / B
            yaw += omega_actual * dt
            yaw = normalize_angle(yaw)
            
            # Record history
            x_history.append(x)
            y_history.append(y)
            yaw_history.append(yaw)
            v_history.append(v)
            v_left_history.append(v_left)
            v_right_history.append(v_right)
            target_history.append(target_idx)
            distance_history.append(distance)
            cte_history.append(cte)
            
            # Record reached waypoints (actual visible waypoints only)
            if i > 0 and target_idx != target_history[-2]:
                reached_idx = target_idx - 1
                
                # Don't record the extension waypoint
                if reached_idx == len(waypoints) - 2:
                    reached_idx = len(waypoints) - 2  # Mark the original final waypoint
                
                if reached_idx not in reached_waypoints:
                    reached_waypoints.append(reached_idx)
            
            # Write debug info
            f.write(f"Iteration {i}: x={x}, y={y}, yaw={yaw}, v={v}, v_left={v_left:.2f}, "
                   f"v_right={v_right:.2f}, target_idx={target_idx}, distance_to_target={distance}, "
                   f"cte={cte:.4f}, heading_error={np.degrees(heading_error):.2f}°\n")
            
            # Check if we've reached the final waypoint
            if target_idx == len(waypoints) - 1 and distance < waypoint_reach_threshold:
                print(f"✓✓ REACHED FINAL WAYPOINT at iteration {i}")
                # Add the last visible waypoint to reached_waypoints (not the invisible extension)
                if len(waypoints) - 2 not in reached_waypoints:
                    reached_waypoints.append(len(waypoints) - 2)
                break
    
    # Return simulation results
    return {
        'x': x_history,
        'y': y_history,
        'yaw': yaw_history,
        'v': v_history,
        'v_left': v_left_history,
        'v_right': v_right_history,
        'target': target_history,
        'distance': distance_history,
        'cte': cte_history,
        'reached_waypoints': reached_waypoints
    }

def plot_results(results, waypoints, show_first_only=False):
    """Plot simulation results, but hide the extension waypoint"""
    plt.figure(figsize=(12, 10))
    
    # Plot only the original waypoints (not the extension)
    visible_waypoints = waypoints[:-1]
    
    # Plot waypoint path and vehicle path
    plt.plot(visible_waypoints[:, 0], visible_waypoints[:, 1], 'ro-', label='Waypoint Path')
    plt.plot(results['x'], results['y'], 'b-', linewidth=2, label='Vehicle Path')
    
    # Mark start and end points
    plt.scatter(results['x'][0], results['y'][0], color='green', s=100, label='Start')
    plt.scatter(results['x'][-1], results['y'][-1], color='purple', s=100, label='End')
    
    # Mark reached waypoints
    for idx in results['reached_waypoints']:
        if idx < len(visible_waypoints):  # Only show actual waypoints
            plt.scatter(visible_waypoints[idx, 0], visible_waypoints[idx, 1], 
                      color='lime', s=150, marker='*')
    
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.title('Skid-Steer Stanley Controller - GPS Waypoint Navigation')
    plt.grid(True)
    plt.legend()
    plt.savefig('skid_steer_path.png')
    plt.show()  # Always show this plot
    
    # If we only want to show the first plot, return here
    if show_first_only:
        return
        
    # The rest of your plotting code remains the same...
    # Plot velocity profile
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(results['v'])), results['v'], 'g-', linewidth=2, label='Average Speed')
    plt.plot(range(len(results['v_left'])), results['v_left'], 'b-', linewidth=1.5, label='Left Wheel')
    plt.plot(range(len(results['v_right'])), results['v_right'], 'r-', linewidth=1.5, label='Right Wheel')
    plt.xlabel('Iteration')
    plt.ylabel('Velocity (m/s)')
    plt.title('Wheel Velocity Profiles')
    plt.grid(True)
    plt.legend()
    plt.savefig('skid_steer_velocity_profile.png')
    #plt.show()
    
    # Plot distance to target
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(results['distance'])), results['distance'], 'r-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('Distance to Target (m)')
    plt.title('Distance to Target vs. Iteration')
    plt.grid(True)
    plt.savefig('skid_steer_distance_profile.png')
    #plt.show()
    
    # Plot cross-track error
    plt.figure(figsize=(12, 6))
    plt.plot(range(len(results['cte'])), results['cte'], 'm-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('Cross-Track Error (m)')
    plt.title('Cross-Track Error vs. Iteration')
    plt.grid(True)
    plt.savefig('skid_steer_cte_profile.png')
    #plt.show()
    
    # Plot wheel speed difference (turn intensity)
    plt.figure(figsize=(12, 6))
    v_diff = [results['v_right'][i] - results['v_left'][i] for i in range(len(results['v_left']))]
    plt.plot(range(len(v_diff)), v_diff, 'c-', linewidth=2)
    plt.xlabel('Iteration')
    plt.ylabel('Wheel Speed Difference (m/s)')
    plt.title('Turning Intensity (Right - Left Wheel Speed)')
    plt.grid(True)
    plt.savefig('skid_steer_turn_intensity.png')
    #plt.show()

def create_animation(results, waypoints):
    """Create animation of vehicle movement"""
    print("Creating animation...")
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Only show original waypoints
    visible_waypoints = waypoints[:-1]
    plt.plot(visible_waypoints[:, 0], visible_waypoints[:, 1], 'ro-', label='Waypoint Path')
    
    # Path line and vehicle marker
    path_line, = plt.plot([], [], 'b-', linewidth=2, label='Vehicle Path')
    vehicle, = plt.plot([], [], 'bo', markersize=8)
    
    # Vehicle visualization - make vehicle wider than long for proper skid-steer look
    vehicle_width = 0.00003   # Width of vehicle in plot coordinates (wider)
    vehicle_length = 0.00002  # Length of vehicle in plot coordinates (shorter)
    vehicle_rect = plt.Rectangle((0, 0), vehicle_width, vehicle_length, 
                               fill=True, color='blue', alpha=0.7)

    # Create tracks that run along the sides of the vehicle body
    # For tracks viewed from above, they should be long and narrow
    track_length = vehicle_width * 0.7  # Length of track (along the vehicle width)
    track_width = vehicle_length * 0.2  # Width of track (thinner than vehicle length)

    # Position wheels/tracks along the left and right sides of vehicle
    # Note that we flip width and height to get tracks running along the sides
    vehicle_left_track = plt.Rectangle((0, 0), track_length, track_width, 
                                     fill=True, color='black')
    vehicle_right_track = plt.Rectangle((0, 0), track_length, track_width, 
                                     fill=True, color='black')
    ax.add_patch(vehicle_rect)
    ax.add_patch(vehicle_left_track)
    ax.add_patch(vehicle_right_track)
    
    # Set axis limits with margin
    margin = 0.0001
    x_min = min(min(results['x']), min(visible_waypoints[:, 0])) - margin
    x_max = max(max(results['x']), max(visible_waypoints[:, 0])) + margin
    y_min = min(min(results['y']), min(visible_waypoints[:, 1])) - margin
    y_max = max(max(results['y']), max(visible_waypoints[:, 1])) + margin
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
        
        # Update vehicle rectangle position
        x_pos = results['x'][idx] - vehicle_width/2
        y_pos = results['y'][idx] - vehicle_length/2
        angle_deg = np.degrees(results['yaw'][idx])
        
        # Transform for rectangle
        t = plt.matplotlib.transforms.Affine2D().rotate_deg_around(
            results['x'][idx], results['y'][idx], angle_deg) + ax.transData
        
        vehicle_rect.set_xy((x_pos, y_pos))
        vehicle_rect.set_transform(t)
        
        # Position tracks on the left and right sides of the vehicle body
        left_x = x_pos + (vehicle_width - track_length)/2  # Center horizontally
        left_y = y_pos - track_width/2                     # Along top edge
        vehicle_left_track.set_xy((left_x, left_y))
        vehicle_left_track.set_transform(t)

        right_x = x_pos + (vehicle_width - track_length)/2  # Center horizontally  
        right_y = y_pos + vehicle_length - track_width/2    # Along bottom edge
        vehicle_right_track.set_xy((right_x, right_y))
        vehicle_right_track.set_transform(t)
        
        # Update heading arrow
        arrow_x = [results['x'][idx], 
                  results['x'][idx] + arrow_length * np.cos(results['yaw'][idx])]
        arrow_y = [results['y'][idx], 
                  results['y'][idx] + arrow_length * np.sin(results['yaw'][idx])]
        heading_arrow.set_data(arrow_x, arrow_y)
        
        # Update status text - adjust target display to hide extension waypoint
        target = results['target'][idx]
        if target >= len(visible_waypoints):
            target = len(visible_waypoints) - 1  # Show as targeting the last visible waypoint
            
        status_text.set_text(
            f'Frame: {idx}/{len(results["x"])-1}\n'
            f'Target: {target}\n'
            f'Heading: {np.degrees(results["yaw"][idx]):.1f}°\n'
            f'Speed: {results["v"][idx]:.2f} m/s\n'
            f'Left: {results["v_left"][idx]:.2f} m/s\n'
            f'Right: {results["v_right"][idx]:.2f} m/s\n'
            f'CTE: {results["cte"][idx]:.2f}m'
        )
        
        return path_line, vehicle, heading_arrow, status_text, vehicle_rect, vehicle_left_track, vehicle_right_track
    
    ani = FuncAnimation(fig, update, frames=len(frame_indices), interval=50, blit=True)
    
    plt.title('Skid-Steer Stanley Controller Animation')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()