import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging
from geopy.distance import geodesic

logger = logging.getLogger("rover.simulation")

def haversine_distance(coord1, coord2):
    """Calculate the great circle distance between two points in meters"""
    return geodesic(coord1, coord2).meters

def normalize_angle(angle):
    """Normalize an angle to [-pi, pi]"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def simplified_stanley_control(x, y, yaw, waypoints, target_idx, max_steer, steering_sensitivity, waypoint_reach_threshold, METERS_PER_LAT_DEGREE):
    """
    Simplified Stanley control using tanh function for steering.
    """
    # Find nearest waypoint
    dist_to_waypoints = np.sum((waypoints[:, :2] - [x, y]) ** 2, axis=1)
    curr_nearest_point_idx = np.argmin(dist_to_waypoints)
    
    # Check if we've reached the target waypoint
    if target_idx < len(waypoints) - 1:
        tx, ty = waypoints[target_idx]
        dist_to_target = np.sqrt((tx - x) ** 2 + (ty - y) ** 2)
        
        if dist_to_target < waypoint_reach_threshold:
            target_idx += 1
    
    # Current target waypoint
    tx = waypoints[target_idx, 0]
    ty = waypoints[target_idx, 1]
    
    # Calculate distance to current target
    dist_to_target = np.sqrt((tx - x) ** 2 + (ty - y) ** 2)
    
    # Calculate bearing to target
    target_bearing = np.arctan2(ty - y, tx - x)
    
    # Calculate heading error
    heading_error = target_bearing - yaw
    
    # Normalize to [-pi, pi]
    while heading_error > np.pi:
        heading_error -= 2 * np.pi
    while heading_error < -np.pi:
        heading_error += 2 * np.pi
    
    # Calculate cross-track error for reporting only
    cte = 0.0  # Simplified: we don't use this for steering
    
    # SIMPLIFIED STEERING WITH TANH
    steering_angle = np.tanh(heading_error / steering_sensitivity)
    delta = steering_angle * max_steer
    
    return delta, target_idx, cte, heading_error, dist_to_target

def simulate(waypoints, x0, y0, yaw0, v0, target_idx=1, target_speed=1.0, 
             dt=0.1, max_steer=np.radians(45), steering_sensitivity=np.pi/3,
             waypoint_reach_threshold=1.0, METERS_PER_LAT_DEGREE=111000):
    """
    Simulate vehicle motion with the simplified tanh-based controller.
    """
    # State variables
    x = x0
    y = y0
    yaw = yaw0
    v = v0
    
    # For storing results
    times = [0.0]
    xs = [x]
    ys = [y]
    yaws = [yaw]
    vs = [v]
    targets = [target_idx]
    steers = [0.0]
    ctes = [0.0]
    
    # Simulation loop
    for i in range(1000):
        # Calculate meters per lon degree based on current latitude
        meters_per_lon_degree = METERS_PER_LAT_DEGREE * np.cos(np.radians(x))
        
        # Simplified Stanley control
        delta, target_idx, cte, heading_error, dist_to_target = simplified_stanley_control(
            x, y, yaw, waypoints, target_idx, max_steer, steering_sensitivity, waypoint_reach_threshold, METERS_PER_LAT_DEGREE)
        
        # Update velocity
        v += 0.5 * (target_speed - v) * dt  # Simple P controller
        v = max(0.0, v)  # Ensure non-negative speed
        
        # Update position using bicycle model
        x += v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
        y += v * np.sin(yaw) * dt / meters_per_lon_degree
        yaw += v * np.tan(delta) * dt / 1.0
        yaw = normalize_angle(yaw)
        
        # Store results
        times.append(times[-1] + dt)
        xs.append(x)
        ys.append(y)
        yaws.append(yaw)
        vs.append(v)
        targets.append(target_idx)
        steers.append(delta)
        ctes.append(cte)
        
        # Stop if we've reached the final waypoint
        if target_idx == len(waypoints) - 1 and dist_to_target < waypoint_reach_threshold:
            logger.info("Reached final waypoint")
            break
    
    return {
        'time': times,
        'x': xs,
        'y': ys,
        'yaw': yaws,
        'v': vs,
        'target_idx': targets,
        'steer': steers,
        'cte': ctes
    }

def plot_results(results, waypoints):
    """Plot the simulation results"""
    plt.figure(figsize=(15, 10))
    
    # Plot path and waypoints
    plt.subplot(2, 2, 1)
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'b-', label='Path')
    plt.plot(waypoints[:, 1], waypoints[:, 0], 'r.', label='Waypoints')
    plt.plot(results['y'], results['x'], 'g-', label='Rover')
    plt.legend()
    plt.title('Path and Trajectory')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.axis('equal')
    
    # Plot steering angle
    plt.subplot(2, 2, 2)
    plt.plot(results['time'], np.degrees(results['steer']), 'r-')
    plt.title('Steering Angle')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.grid(True)
    
    # Plot velocity
    plt.subplot(2, 2, 3)
    plt.plot(results['time'], results['v'], 'b-')
    plt.title('Velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    
    # Plot cross-track error
    plt.subplot(2, 2, 4)
    plt.plot(results['time'], results['cte'], 'k-')
    plt.title('Cross-Track Error')
    plt.xlabel('Time [s]')
    plt.ylabel('CTE [m]')
    plt.grid(True)
    
    plt.tight_layout()
    plt.savefig('simulation_results.png')
    plt.show()

def create_animation(results, waypoints):
    """Create an animation of the rover's movement"""
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Plot the waypoints and path
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'r-', label='Path')
    ax.plot(waypoints[:, 1], waypoints[:, 0], 'k.', markersize=8, label='Waypoints')
    
    # Rover marker
    rover, = ax.plot([], [], 'go', markersize=10, label='Rover')
    
    # Rover direction indicator
    direction, = ax.plot([], [], 'g-', linewidth=2)
    
    # Set plot limits
    x_min, x_max = min(waypoints[:, 1]) - 0.0005, max(waypoints[:, 1]) + 0.0005
    y_min, y_max = min(waypoints[:, 0]) - 0.0005, max(waypoints[:, 0]) + 0.0005
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    
    ax.set_xlabel('Longitude')
    ax.set_ylabel('Latitude')
    ax.set_title('Rover Simulation')
    ax.legend()
    ax.grid(True)
    
    def update(frame):
        if frame < len(results['x']):
            # Update rover position
            rover.set_data(results['y'][frame], results['x'][frame])
            
            # Update direction indicator
            yaw = results['yaw'][frame]
            arrow_len = 0.0002  # Length of the direction indicator
            dx = arrow_len * np.sin(yaw)
            dy = arrow_len * np.cos(yaw)
            direction.set_data([results['y'][frame], results['y'][frame] + dx],
                               [results['x'][frame], results['x'][frame] + dy])
        return rover, direction
    
    ani = FuncAnimation(fig, update, frames=len(results['x']), interval=50, blit=True)
    ani.save('rover_animation.gif', writer='pillow', fps=20)
    plt.show()