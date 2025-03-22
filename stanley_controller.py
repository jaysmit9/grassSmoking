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

# BULLDOZER MODE - Extreme parameter settings
k = 2.0  # Reduced steering gain for smoother movements
Kp = 2.0  # Doubled speed gain for faster acceleration
dt = 0.1  # Time step
L = 0.5   # Wheel base of vehicle for skid-steer (meters)

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

# BULLDOZER MODE - Speed and thresholds
cruising_speed = 3  # MUCH higher speed
turning_speed = 1.0   # Increased from 0.5 for more effective turns
waypoint_reach_threshold = 1.0  # MUCH larger threshold for reaching waypoints
turn_threshold = np.radians(20)
stuck_timeout = 30  # Number of iterations before considering "stuck"

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

# Fixed bulldozer controller with proper progress tracking

def bulldozer_control(x, y, yaw, v, waypoints, target_idx, stuck_counter=0, prev_positions=None):
    """
    A controller that focuses 100% on making forward progress and preventing getting stuck.
    It sacrifices path accuracy for guaranteed forward movement.
    """
    if prev_positions is None:
        prev_positions = []
        
    # Get current target waypoint
    target_waypoint = waypoints[target_idx]
    
    # Calculate desired heading to target
    dx = target_waypoint[0] - x
    dy = target_waypoint[1] - y
    desired_yaw = np.arctan2(dy, dx)
    
    # Calculate heading error
    heading_error = normalize_angle(desired_yaw - yaw)
    
    # Calculate distance to target waypoint
    distance_to_target = haversine_distance((x, y), (target_waypoint[0], target_waypoint[1]))
    print(f"Bulldozer: Distance to waypoint {target_idx}: {distance_to_target:.1f}m, error: {np.degrees(heading_error):.1f}°")

    # IMPROVED STUCK DETECTION - Check if we're making progress toward target
    is_stuck = False
    
    # We need at least 20 positions to detect if we're stuck
    if len(prev_positions) >= 20:
        # Calculate progress toward the waypoint in last 20 iterations
        old_distance = haversine_distance((prev_positions[-20][0], prev_positions[-20][1]), 
                                         (target_waypoint[0], target_waypoint[1]))
        progress = old_distance - distance_to_target  # Positive means getting closer
        
        # Calculate total movement distance in last 20 iterations
        total_movement = 0
        for i in range(len(prev_positions)-1, len(prev_positions)-20, -1):
            total_movement += haversine_distance((prev_positions[i-1][0], prev_positions[i-1][1]),
                                               (prev_positions[i][0], prev_positions[i][1]))
        
        # If we're moving but not making progress toward target
        if total_movement > 1.0 and progress < 0.2:  # Moving but not getting closer
            stuck_counter += 1
            print(f"⚠️ LOW PROGRESS: Moved {total_movement:.3f}m but got {progress:.3f}m closer to target, counter: {stuck_counter}/{stuck_timeout}")
            if stuck_counter > stuck_timeout:
                is_stuck = True
                print("🚨 STUCK DETECTED - EXECUTING EMERGENCY MANEUVER 🚨")
        else:
            stuck_counter = max(0, stuck_counter - 2)  # More aggressively reduce counter when progress is good
            print(f"✓ Good progress: {progress:.3f}m closer to target in last 20 iterations")

    # WAYPOINT REACHED CHECK - Very generous threshold
    if distance_to_target < waypoint_reach_threshold:
        next_target_idx = min(target_idx + 1, len(waypoints) - 1)
        if next_target_idx != target_idx:
            print(f"✓ REACHED waypoint {target_idx}, moving to waypoint {next_target_idx}")
            
            # Calculate new heading to next waypoint
            next_waypoint = waypoints[next_target_idx]
            dx_next = next_waypoint[0] - x
            dy_next = next_waypoint[1] - y
            desired_yaw_next = np.arctan2(dy_next, dx_next)
            
            # Calculate how much we need to turn
            turn_angle = normalize_angle(desired_yaw_next - yaw)
            print(f"Need to turn {np.degrees(turn_angle):.1f}° to face waypoint {next_target_idx}")
            
            # Return a strong steering command based on the turn angle
            # This ensures we immediately start turning toward the new waypoint
            steer_max = np.pi/3  # Limit to 60 degrees max steering
            steer_cmd = max(min(turn_angle * 2.0, steer_max), -steer_max)
            
            # Slow down during the turn
            turn_speed = turning_speed * 0.5
            
            return steer_cmd, turn_speed, next_target_idx, 0, heading_error

    # Special handling for tough transitions (like waypoint 3)
    if target_idx == 3 and abs(heading_error) > np.radians(60):
        print(f"SPECIAL HANDLING FOR WAYPOINT 3: Heading error {np.degrees(heading_error):.1f}°")
        
        # Execute a specialized turn pattern for this difficult waypoint
        turn_dir = np.sign(heading_error)
        
        # More aggressive turning for waypoint 3
        return turn_dir * np.pi/2, 0.4, target_idx, stuck_counter, heading_error  # Increased speed and angle
    
    # Only skip waypoint if we're truly stuck (not just after some iterations)
    elif is_stuck:
        next_target_idx = min(target_idx + 1, len(waypoints) - 1)
        if next_target_idx != target_idx:
            print(f"🚜 BULLDOZER: SKIPPING waypoint {target_idx} due to being truly stuck!")
            return np.pi/4, cruising_speed, next_target_idx, 0, heading_error  # Turn 45° and move at full speed

    # EMERGENCY ESCAPE - If stuck, make a dramatic turn and accelerate
    if is_stuck:
        # Choose random direction to break symmetry
        turn_dir = 1 if np.random.random() > 0.5 else -1
        return turn_dir * np.pi/2, cruising_speed, target_idx, 0, heading_error  # 90° turn at full speed
        
    # NORMAL STEERING - Enhanced for extreme angles
    if abs(heading_error) > np.radians(90):  # Heading very wrong (more than 90°)
        # Use stronger steering for very sharp turns
        steer_angle = np.sign(heading_error) * np.pi/2  # Maximum steering
        target_speed = turning_speed * 0.3  # Very slow speed for extreme turns
        print(f"EXTREME TURN: {np.degrees(heading_error):.1f}° - Using maximum steering")
    else:
        # Normal proportional steering
        steer_angle = k * heading_error
        
        # SPEED CONTROL - Focus on maximum forward progress
        if abs(heading_error) > np.radians(60):  # Heading quite wrong
            target_speed = turning_speed * 0.5  # Half turning speed
        elif abs(heading_error) > np.radians(30):  # Heading somewhat wrong
            target_speed = turning_speed * 0.8  # 80% turning speed
        else:
            target_speed = cruising_speed  # Full speed ahead!
    
    # Limit maximum steering angle
    steer_angle = max(min(steer_angle, np.pi/2), -np.pi/2)
    
    return steer_angle, target_speed, target_idx, stuck_counter, heading_error

# Update the simulation loop to track position history

# Simulation loop with bulldozer controller
target_idx = 1
x_history = [x]
y_history = [y]
yaw_history = [yaw]
v_history = [v]
target_history = [target_idx]
reached_waypoints = [0]  # Start with the first waypoint as reached
stuck_counter = 0
prev_positions = [(x, y)]  # Track position history for better stuck detection

with open('debug_output.txt', 'w') as f:
    for i in range(1500):
        if i % 50 == 0:
            print(f"Processing iteration {i}...")
            
        # Run the bulldozer controller with position history
        steer_angle, target_speed, next_target_idx, stuck_counter, heading_error = bulldozer_control(
            x, y, yaw, v, waypoints, target_idx, stuck_counter, prev_positions
        )
        
        # Track waypoint changes
        if next_target_idx != target_idx:
            print(f"⚠️ WAYPOINT CHANGED from {target_idx} to {next_target_idx} at iteration {i}")
            # Reset counter when changing targets
            stuck_counter = 0
            
        target_idx = next_target_idx
        
        # Speed control with higher gain
        v += Kp * (target_speed - v) * dt
        
        # CRITICAL: Update position using proper geographic conversions
        lat_change = v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
        lon_change = v * np.sin(yaw) * dt / meters_per_lon_degree(x)
        
        # FORCE MINIMUM MOVEMENT - bulldozer must always move forward
        min_movement = 0.005  # Significant minimum movement in meters
        min_lat = min_movement / METERS_PER_LAT_DEGREE
        min_lon = min_movement / meters_per_lon_degree(x)
        
        if v > 0.1:  # Only enforce when actually trying to move
            # Ensure minimum movement in yaw direction
            if abs(lat_change) < min_lat:
                lat_change = np.sign(np.cos(yaw)) * min_lat
            if abs(lon_change) < min_lon:
                lon_change = np.sign(np.sin(yaw)) * min_lon
        
        # Update position
        x += lat_change
        y += lon_change
        
        # Store position history
        prev_positions.append((x, y))
        if len(prev_positions) > 100:  # Keep last 100 positions
            prev_positions = prev_positions[-100:]
        
        # ENHANCED TURNING - Maximum turning capacity at sharp angles
        turning_boost = 1.0

        # Apply super-aggressive turning when the heading error is extreme
        if abs(heading_error) > np.radians(80):
            turning_boost = 60.0  # Increased from 40.0 for even more extreme turning
            v = max(v, 0.5)  # Increased minimum speed during turns
            print(f"CRITICAL TURN: {np.degrees(heading_error):.1f}° - Using maximum turn boost")
        elif abs(heading_error) > np.radians(45):  # Large turn needed
            turning_boost = 20.0  # High turning capability
        elif v < 1.0:  # Low speed, standard boost
            turning_boost = 10.0  # Original turning capability

        # Calculate yaw change
        yaw_change = v * np.tan(steer_angle) * turning_boost / L * dt

        # FORCE MINIMUM TURN RATE for large heading errors
        if abs(heading_error) > np.radians(45) and abs(yaw_change) < 0.05:  # Less than ~3 degrees
            yaw_change = 0.05 * np.sign(heading_error)  # Force minimum turn rate
            print(f"Forcing minimum turn rate: {np.degrees(yaw_change):.1f}° per iteration")

        yaw += yaw_change
        yaw = normalize_angle(yaw)
        
        # Record history
        x_history.append(x)
        y_history.append(y)
        yaw_history.append(yaw)  # Change from 'y' to 'yaw'
        v_history.append(v)
        target_history.append(target_idx)
        
        # Record reached waypoints when target changes
        if i > 0 and target_idx != target_history[-2]:
            reached_waypoints.append(target_idx - 1)
        
        # Write debugging info
        f.write(f"Iteration {i}: x={x}, y={y}, yaw={yaw}, v={v}, steer_angle={steer_angle}, " +
                f"target_idx={target_idx}, distance_to_target={haversine_distance((x, y), (waypoints[target_idx][0], waypoints[target_idx][1]))}\n")
        
        # Check if we've reached the final waypoint
        if target_idx == len(waypoints) - 1 and haversine_distance((x, y), (waypoints[-1][0], waypoints[-1][1])) < waypoint_reach_threshold:
            print(f"✓✓ REACHED FINAL WAYPOINT at iteration {i}")
            reached_waypoints.append(target_idx)
            break

# Create visualization showing waypoints reached
plt.figure(figsize=(12, 10))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')
plt.plot(x_history, y_history, 'b-', linewidth=2, label='Rover Path')
plt.scatter(x_history[0], y_history[0], color='green', s=100, label='Start')
plt.scatter(x_history[-1], y_history[-1], color='purple', s=100, label='End')

# Mark reached waypoints
for idx in reached_waypoints:
    if idx < len(waypoints):
        plt.scatter(waypoints[idx, 0], waypoints[idx, 1], color='lime', s=150, marker='*')  # Using star instead of checkmark

plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.title('Bulldozer Mode - Progress Over Accuracy')
plt.grid(True)
plt.legend()
plt.savefig('bulldozer_path.png')
plt.show()

# Create animation for visual debugging
print("Creating animation...")
fig, ax = plt.subplots(figsize=(12, 10))
plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro-', label='Waypoint Path')

# Start with empty path that will grow
path_line, = plt.plot([], [], 'b-', linewidth=2, label='Rover Path')
rover, = plt.plot([], [], 'bo', markersize=8)

# Set proper axis limits with margin
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

# Optional enhancement: Show rover orientation in animation
def update(i):
    idx = frame_indices[i]
    path_line.set_data(x_history[:idx+1], y_history[:idx+1])
    
    # Plot the rover as a point
    rover.set_data([x_history[idx]], [y_history[idx]])
    
    # You could add an arrow to show orientation
    # (requires importing matplotlib.patches and adding an arrow to the plot)
    
    target = target_history[idx] if idx < len(target_history) else target_history[-1]
    status_text.set_text(f'Frame: {idx}/{len(x_history)-1}\nTarget: {target}')
    return path_line, rover, status_text

ani = FuncAnimation(fig, update, frames=len(frame_indices), interval=50, blit=True)

plt.title('Bulldozer Mode Animation')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()