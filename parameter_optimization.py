import numpy as np
import json
import matplotlib.pyplot as plt
from geopy.distance import geodesic
import time
from itertools import product
import pandas as pd
from tqdm import tqdm
import multiprocessing as mp

# Import the necessary functions from the controller
from Skid_Steer_Stanley_Controller import (
    add_extension_waypoint,
    haversine_distance,
    normalize_angle,
    meters_per_lon_degree,
    stanley_control
)

def evaluate_parameters(params):
    """
    Run a simulation with the given parameters and return performance metrics
    
    Args:
        params: tuple of (k, k_e, turning_factor)
        
    Returns:
        dict of performance metrics
    """
    k, k_e, turning_factor = params
    
    # Set B as a constant
    B = 1.1  # Fixed track width value
    
    # Load waypoints
    with open('polygon_data.json') as f:
        polygon_data = json.load(f)
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    waypoints = add_extension_waypoint(waypoints, 1.0)
    
    # Initial state
    x = waypoints[0, 0]
    y = waypoints[0, 1]
    v = 0.2
    
    # Set initial yaw to face the first waypoint
    dx = waypoints[1, 0] - x
    dy = waypoints[1, 1] - y
    yaw = np.arctan2(dy, dx)
    
    # Constants
    dt = 0.1
    target_speed = 2.0
    target_idx = 1
    waypoint_reach_threshold = 1.0
    METERS_PER_LAT_DEGREE = 111000
    max_iter = 1000  # Limit to avoid infinite loops
    L = 1.0  # Wheelbase (if needed in Stanley control)
    
    # Simulation variables
    x_history = [x]
    y_history = [y]
    v_history = [v]
    cte_history = [0]
    
    # Run simulation
    reached_final = False
    for i in range(max_iter):
        # Get steering command
        delta, target_idx, distance, cte, heading_error = stanley_control(
            x, y, yaw, v, waypoints, target_idx, k, k_e, waypoint_reach_threshold
        )
        
        # Limit steering angle
        max_steer = np.radians(50)  # Maximum steering angle
        delta = np.clip(delta, -max_steer, max_steer)
        
        # Update velocity with simple proportional control
        dv = 0.5 * (target_speed - v)  # Simple proportional control
        v += dv * dt
        
        # Update position
        lat_change = v * np.cos(yaw) * dt / METERS_PER_LAT_DEGREE
        lon_change = v * np.sin(yaw) * dt / meters_per_lon_degree(x)
        x += lat_change
        y += lon_change
        
        # Update heading with appropriate turning factor
        yaw += v * np.tan(delta) * turning_factor / L * dt
        yaw = normalize_angle(yaw)
        
        # Record history
        x_history.append(x)
        y_history.append(y)
        v_history.append(v)
        cte_history.append(cte)
        
        # Check for completion
        if target_idx == len(waypoints) - 1 and distance < waypoint_reach_threshold:
            reached_final = True
            break
    
    # Calculate performance metrics
    avg_cte = np.mean(np.abs(cte_history))
    max_cte = np.max(np.abs(cte_history))
    
    # Calculate path smoothness (lower is better)
    path_length = 0
    for j in range(1, len(x_history)):
        path_length += haversine_distance(
            (x_history[j-1], y_history[j-1]),
            (x_history[j], y_history[j])
        )
    
    # Calculate ideal path length
    ideal_length = 0
    for j in range(1, len(waypoints)):
        ideal_length += haversine_distance(
            (waypoints[j-1][0], waypoints[j-1][1]),
            (waypoints[j][0], waypoints[j][1])
        )
    
    # Path efficiency (closer to 1 is better)
    path_efficiency = ideal_length / path_length if path_length > 0 else 0
    
    # Calculate oscillation metric
    cte_changes = np.diff(np.array(cte_history))
    oscillation = np.sum(np.abs(np.diff(np.sign(cte_changes)))) if len(cte_changes) > 1 else 0
    
    # Calculate completion score
    completion = 1.0 if reached_final else distance / haversine_distance(
        (x, y), (waypoints[-1][0], waypoints[-1][1])
    )
    
    # Calculate time to complete (iterations)
    completion_time = len(x_history)
    
    # Combine metrics into a single score (weighted)
    score = (
        0.4 * (1 - min(1, avg_cte)) +  # Lower CTE is better
        0.2 * path_efficiency +         # Higher efficiency is better
        0.2 * (1 - oscillation/100) +   # Lower oscillation is better
        0.2 * completion                # Higher completion is better
    )
    
    return {
        'k': k,
        'k_e': k,
        'turning_factor': turning_factor,
        'avg_cte': avg_cte,
        'max_cte': max_cte,
        'path_efficiency': path_efficiency,
        'oscillation': oscillation,
        'completion': completion,
        'completion_time': completion_time,
        'score': score,
        'path': (x_history, y_history),
        'cte': cte_history,
        'velocity': v_history
    }

def run_parameter_sweep(parallel=True):
    """Run a grid search over parameter space"""
    # Define parameter ranges
    k_range = [1.5, 2.0, 2.8, 3.5, 4.2]
    k_e_range = [3.0, 4.0, 5.0, 6.0, 7.0]
    turning_factor_range = [1.0, 2.0, 3.0, 4.0]
    
    # Generate all parameter combinations
    param_combinations = list(product(
        k_range, k_e_range, turning_factor_range
    ))
    
    print(f"Running {len(param_combinations)} parameter combinations...")
    
    if parallel:
        # Use multiprocessing to speed up the search
        with mp.Pool(processes=mp.cpu_count()) as pool:
            results = list(tqdm(
                pool.imap(evaluate_parameters, param_combinations),
                total=len(param_combinations)
            ))
    else:
        # Run sequentially
        results = []
        for params in tqdm(param_combinations):
            results.append(evaluate_parameters(params))
    
    # Convert to DataFrame for analysis
    results_df = pd.DataFrame(results)
    
    # Sort by score (higher is better)
    results_df = results_df.sort_values('score', ascending=False)
    
    # Save results
    results_df.to_csv('parameter_optimization_results.csv', index=False)
    
    return results_df

def visualize_best_parameters(results_df, top_n=5):
    """Visualize the top N parameter combinations"""
    # Get the top N parameter combinations
    top_results = results_df.head(top_n)
    
    print("\n===== Top Parameter Combinations =====")
    for i, row in top_results.iterrows():
        print(f"\nRank {i+1} (Score: {row['score']:.4f}):")
        print(f"k = {row['k']}, k_e = {row['k']}, turning_factor = {row['turning_factor']}")
        print(f"Avg CTE: {row['avg_cte']:.4f}, Completion: {row['completion']:.2f}")
        print(f"Path Efficiency: {row['path_efficiency']:.4f}, Oscillation: {row['oscillation']:.1f}")
    
    # Plot the paths of the top 3 parameter combinations
    plt.figure(figsize=(12, 8))
    
    # Load waypoints
    with open('polygon-to-path.json') as f:
        polygon_data = json.load(f)
    waypoints = np.array([(point['lat'], point['lon']) for point in polygon_data])
    
    # Plot waypoints
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ko-', label='Waypoints')
    
    # Plot paths for top 3 combinations
    colors = ['r', 'g', 'b']
    for i, (_, row) in enumerate(top_results.head(3).iterrows()):
        x_hist, y_hist = row['path']
        plt.plot(x_hist, y_hist, f'{colors[i]}:', linewidth=2, 
                 label=f"k={row['k']}, k_e={row['k']}, turning_factor = {row['turning_factor']}")
    
    plt.title('Top 3 Parameter Combinations')
    plt.xlabel('Latitude')
    plt.ylabel('Longitude')
    plt.legend()
    plt.grid(True)
    plt.savefig('top_parameter_paths.png')
    plt.show()
    
    # Plot CTE for top 3 combinations
    plt.figure(figsize=(12, 6))
    for i, (_, row) in enumerate(top_results.head(3).iterrows()):
        plt.plot(row['cte'], f'{colors[i]}:', linewidth=2, 
                 label=f"k={row['k']}, k_e={row['k_e']}, turning_factor = {row['turning_factor']}")
    
    plt.title('Cross-Track Error for Top 3 Parameter Combinations')
    plt.xlabel('Iteration')
    plt.ylabel('CTE (m)')
    plt.legend()
    plt.grid(True)
    plt.savefig('top_parameter_cte.png')
    plt.show()
    
    # Plot velocity profiles
    plt.figure(figsize=(12, 6))
    for i, (_, row) in enumerate(top_results.head(3).iterrows()):
        plt.plot(row['velocity'], f'{colors[i]}:', linewidth=2, 
                 label=f"k={row['k']}, k_e={row['k_e']}, turning_factor = {row['turning_factor']}")
    
    plt.title('Velocity Profiles for Top 3 Parameter Combinations')
    plt.xlabel('Iteration')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig('top_parameter_velocity.png')
    plt.show()
    
    # Create parameter heatmaps
    plt.figure(figsize=(15, 10))
    plt.subplot(2, 2, 1)
    pivot = results_df.pivot_table(values='score', index='k', columns='k_e', aggfunc='mean')
    plt.imshow(pivot, cmap='viridis', aspect='auto', interpolation='nearest')
    plt.colorbar(label='Score')
    plt.title('k vs k_e')
    plt.xlabel('k_e index')
    plt.ylabel('k index')
    
    plt.subplot(2, 2, 2)
    pivot = results_df.pivot_table(values='score', index='turning_factor', columns='k', aggfunc='mean')
    plt.imshow(pivot, cmap='viridis', aspect='auto', interpolation='nearest')
    plt.colorbar(label='Score')
    plt.title('turning_factor vs k')
    plt.xlabel('k index')
    plt.ylabel('turning_factor index')
    
    plt.subplot(2, 2, 3)
    pivot = results_df.pivot_table(values='avg_cte', index='k', columns='k_e', aggfunc='mean')
    plt.imshow(pivot, cmap='coolwarm_r', aspect='auto', interpolation='nearest')
    plt.colorbar(label='Avg CTE')
    plt.title('k vs k_e (Avg CTE)')
    plt.xlabel('k_e index')
    plt.ylabel('k index')
    
    plt.subplot(2, 2, 4)
    pivot = results_df.pivot_table(values='oscillation', index='k', columns='k_e', aggfunc='mean')
    plt.imshow(pivot, cmap='coolwarm_r', aspect='auto', interpolation='nearest')
    plt.colorbar(label='Oscillation')
    plt.title('k vs k_e (Oscillation)')
    plt.xlabel('k_e index')
    plt.ylabel('k index')
    
    plt.tight_layout()
    plt.savefig('parameter_heatmaps.png')
    plt.show()

def main():
    print("Starting parameter optimization for skid-steer controller...")
    print("Track width (B) fixed at 1.1 meters")
    start_time = time.time()
    
    # Run parameter sweep
    results = run_parameter_sweep(parallel=True)
    
    # Get best parameters
    best_params = results.iloc[0]
    print("\n===== Best Parameters =====")
    print(f"k = {best_params['k']}")
    print(f"k_e = {best_params['k_e']}")
    print(f"turning_factor = {best_params['turning_factor']}")
    print(f"Score: {best_params['score']:.4f}")
    
    # Visualize results
    visualize_best_parameters(results)
    
    print(f"\nOptimization completed in {time.time() - start_time:.1f} seconds")
    
    # Return the optimal parameters for use in the main controller
    return {
        'k': best_params['k'],
        'k_e': best_params['k_e'],
        'turning_factor': best_params['turning_factor']
    }

if __name__ == "__main__":
    optimal_params = main()
    
    # Write optimal parameters to a file for later use
    with open('optimal_parameters.json', 'w') as f:
        json.dump(optimal_params, f, indent=4)