import numpy as np
import matplotlib.pyplot as plt
import json
from matplotlib.animation import FuncAnimation
from geopy.distance import geodesic

# Load polygon data from JSON file
with open('polygon_data.json') as f:
    polygon_data = json.load(f)

# Extract coordinates from polygon data
waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])

# Controller parameters
k = 2.0        # Stanley controller gain
Kp = 1.0       # Speed control gain
dt = 0.1       # Time step
L = 1.0       # Wheel base of vehicle (meters)

# Constants for geographic coordinate conversion
METERS_PER_LAT_DEGREE = 111000
def meters_per_lon_degree(lat):
    return METERS_PER_LAT_DEGREE * np.cos(np.radians(lat))

# Initial state
x = waypoints[0, 0]
y = waypoints[0, 1]
dx = waypoints[1, 0] - x
dy = waypoints[1, 1] - y
yaw = np.arctan2(dy, dx)
v = 0.0

# Speed and threshold parameters
cruising_speed = 2.0    # Normal cruising speed
turning_speed = 1.0     # Speed when turning
waypoint_reach_threshold = 5.0  # Distance to consider waypoint reached

# Function to calculate the distance between two geographic coordinates
def haversine_distance(coord1, coord2):
    return geodesic(coord1, coord2).meters

def normalize_angle(angle):
    """Normalize angle to be between -pi and pi"""
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle

def stanley_controller(x, y, yaw, v, waypoints, target_idx):
    """
    Basic Stanley controller with adaptations for GPS waypoint navigation
    """
    # Get current target waypoint
    target_waypoint = waypoints[target_idx]
    
    # Calculate desired heading to target
    dx = target_waypoint[0] - x
    dy = target_waypoint[1] - y
    desired_yaw = np.arctan2(dy, dx)
    
    # Calculate heading error with proper normalization
    heading_error = normalize_angle(desired_yaw - yaw)
    
    # Calculate distance to target waypoint
    distance_to_target = haversine_distance((x, y), (target_waypoint[0], target_waypoint[1]))
    print(f"Distance to waypoint {target_idx}: {distance_to_target:.1f}m, error: {np.degrees(heading_error):.1f}°")

    # Check if we've reached the waypoint
    if distance_to_target < waypoint_reach_threshold:
        next_target_idx = min(target_idx + 1, len(waypoints) - 1)
        if next_target_idx != target_idx:
            print(f"✓ Reached waypoint {target_idx}, moving to waypoint {next_target_idx}")
            
            # Calculate heading to next waypoint to prepare proper turn
            next_waypoint = waypoints[next_target_idx]
            dx_next = next_waypoint[0] - x
            dy_next = next_waypoint[1] - y
            desired_yaw_next = np.arctan2(dy_next, dx_next)
            
            # Calculate turn angle needed
            turn_angle = normalize_angle(desired_yaw_next - yaw)
            print(f"Need to turn {np.degrees(turn_angle):.1f}° to face waypoint {next_target_idx}")
            
            # Create appropriate steering command for the turn
            steer_max = np.pi/3  # Limit to 60 degrees
            steer_cmd = max(min(turn_angle * 1.5, steer_max), -steer_max)
            
            # Sharp turn when changing waypoints
            target_speed = turning_speed * 0.8
            
            return steer_cmd, target_speed, next_target_idx
    
    # Normal steering control - use heading error directly
    steer_angle = k * heading_error
    
    # Special handling for the transition to waypoint 3 (which was tough before)
    if target_idx == 3 and abs(heading_error) > np.radians(60):
        print(f"SPECIAL HANDLING FOR WAYPOINT 3: Heading error {np.degrees(heading_error):.1f}°")
        steer_angle = np.sign(heading_error) * np.pi/2  # Maximum steering
        target_speed = turning_speed * 0.5  # Slow down significantly
        return steer_angle, target_speed, target_idx
    
    # Adjust speed based on heading error
    if abs(heading_error) > np.radians(60):
        target_speed = turning_speed * 0.5  # Slow down for sharp turns
    elif abs(heading_error) > np.radians(30):
        target_speed = turning_speed * 0.8  # Moderate speed for moderate turns
    else:
        target_speed = cruising_speed  # Full speed when heading is good
    
    # Limit maximum steering angle
    steer_angle = max(min(steer_angle, np.pi/2), -np.pi/2)
    
    return steer_angle, target_speed, target_idx

# Simulation loop
target_idx = 1
x_history = [x]
y_history = [y]
yaw_history = [yaw]  # FIXED: Store yaw, not y
v_history = [v]
target_history = [target_idx]
reached_waypoints = [0]

with open('debug_output.txt', 'w') as f:
    for i in range(1500):
        if i % 50 == 0:
            print(f"Processing iteration {i}...")
            
        steer_angle, target_speed, next_target_idx = stanley_controller(
            x, y, yaw, v, waypoints, target_idx
        )
        
        # Track waypoint changes
        if next_target_idx != target_idx:
            print(f"⚠️ WAYPOINT CHANGED from {target_idx} to {next_target_idx} at iteration {i}")
            reached_waypoints.append(target_idx)
            
        target_idx = next_target_idx
        
        # Speed control
        v += Kp * (target_speed - v) * dt
        
        # CRITICAL FIX: Update position using proper geographic conversions
        lat_change = v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
        lon_change = v * np.sin(yaw) * dt / meters_per_lon_degree(x)
        
        # Ensure minimum movement when trying to move
        if v > 0.3:  # Only when we're actually trying to move
            min_movement = 0.0005  # 0.5mm minimum movement
            min_lat = min_movement / METERS_PER_LAT_DEGREE
            min_lon = min_movement / meters_per_lon_degree(x)
            
            if abs(lat_change) < min_lat:
                lat_change = np.sign(np.cos(yaw)) * min_lat
            if abs(lon_change) < min_lon:
                lon_change = np.sign(np.sin(yaw)) * min_lon
        
        # Update position
        x += lat_change
        y += lon_change
        
        # Calculate appropriate turning factor based on speed
        turning_factor = 2.0  # Base turning factor

        # Enhanced turning at low speeds or for sharp turns
        if v < 0.5:
            turning_factor = 5.0  # Better turning at low speeds
        elif abs(steer_angle) > np.radians(45):
            turning_factor = 3.0  # Better turning for sharp angles

        # Update yaw (heading) with enhanced turning
        yaw_change = v * np.tan(steer_angle) * turning_factor / L * dt

        # Ensure minimum turn rate for large steering angles
        if abs(steer_angle) > np.radians(30) and abs(yaw_change) < 0.02:
            yaw_change = 0.02 * np.sign(steer_angle)
            print(f"Forcing minimum turn rate: {np.degrees(yaw_change):.1f}° per iteration")
            
        yaw += yaw_change
        yaw = normalize_angle(yaw)
        
        # Record history - FIXED: Store yaw, not y
        x_history.append(x)
        y_history.append(y)
        yaw_history.append(yaw)
        v_history.append(v)
        target_history.append(target_idx)
        
        # Write debugging info
        f.write(f"Iteration {i}: x={x}, y={y}, yaw={yaw}, v={v}, steer_angle={steer_angle}, " +
                f"target_idx={target_idx}, distance_to_target={haversine_distance((x, y), (waypoints[target_idx][0], waypoints[target_idx][1]))}\n")
        
        # Check if we've reached the final waypoint
        if target_idx == len(waypoints) - 1 and haversine_distance((x, y), (waypoints[-1][0], waypoints[-1][1])) < waypoint_reach_threshold:
            print(f"✓✓ REACHED FINAL WAYPOINT at iteration {i}")
            reached_waypoints.append(target_idx)
            break

# Create visualization
plt.figure(figsize=(12, 10))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')
plt.plot(x_history, y_history, 'b-', linewidth=2, label='Rover Path')
plt.scatter(x_history[0], y_history[0], color='green', s=100, label='Start')
plt.scatter(x_history[-1], y_history[-1], color='purple', s=100, label='End')

# Mark reached waypoints
for idx in reached_waypoints:
    if idx < len(waypoints):
        plt.scatter(waypoints[idx, 0], waypoints[idx, 1], color='lime', s=150, marker='*')

plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.title('Stanley Controller - GPS Waypoint Navigation')
plt.grid(True)
plt.legend()
plt.savefig('stanley_path.png')
plt.show()

# Create animation
print("Creating animation...")
fig, ax = plt.subplots(figsize=(12, 10))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')

path_line, = plt.plot([], [], 'b-', linewidth=2, label='Rover Path')
rover, = plt.plot([], [], 'bo', markersize=8)

# Set axis limits
margin = 0.0001
x_min = min(min(x_history), min(waypoints[:, 0])) - margin
x_max = max(max(x_history), max(waypoints[:, 0])) + margin
y_min = min(min(y_history), min(waypoints[:, 1])) - margin
y_max = max(max(y_history), max(waypoints[:, 1])) + margin
plt.xlim(x_min, x_max)
plt.ylim(y_min, y_max)

# Skip frames for smoother animation
skip = max(1, len(x_history) // 200)
frame_indices = list(range(0, len(x_history), skip))
if len(x_history)-1 not in frame_indices:
    frame_indices.append(len(x_history)-1)

# Status text
status_text = plt.text(0.02, 0.95, '', transform=ax.transAxes,
                     bbox=dict(facecolor='white', alpha=0.7))

# Add orientation indicator with arrow
def update(i):
    idx = frame_indices[i]
    path_line.set_data(x_history[:idx+1], y_history[:idx+1])
    rover.set_data([x_history[idx]], [y_history[idx]])
    
    # Display current target and position
    target = target_history[idx] if idx < len(target_history) else target_history[-1]
    status_text.set_text(f'Frame: {idx}/{len(x_history)-1}\nTarget: {target}\nHeading: {np.degrees(yaw_history[idx]):.1f}°')
    
    return path_line, rover, status_text

ani = FuncAnimation(fig, update, frames=len(frame_indices), interval=50, blit=True)

plt.title('Stanley Controller Animation')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()