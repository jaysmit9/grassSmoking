import matplotlib.pyplot as plt
import numpy as np

# Parse the last 100 iterations from debug_output.txt
start_iter = 939
end_iter = 1039

# Make sure all arrays have the same length
x_coords = [
    34.15224406, 34.15224439, 34.15224472, 34.15224505, 34.15224538,
    34.15224571, 34.15224604, 34.15224638, 34.15224671, 34.15224705,
    34.15224738, 34.15224771, 34.15224805, 34.15224839, 34.15224872,
    34.15224906, 34.15224939, 34.15224973, 34.15225007, 34.15225041,
    34.15225074, 34.15225108, 34.15225142, 34.15225176, 34.15225209,
    34.15225243, 34.15225277, 34.15225311, 34.15225345, 34.15225379,
    34.15225413, 34.15225447, 34.15225481, 34.15225515, 34.15225549,
    34.15225583, 34.15225617, 34.15225651, 34.15225685, 34.15225719,
    34.15225754, 34.15225788, 34.15225822, 34.15225856, 34.15225890,
    34.15225924, 34.15225959, 34.15225993, 34.15226027, 34.15226062,
    34.15226096, 34.15226131, 34.15226165, 34.15226199, 34.15226234,
    34.15226268, 34.15226303, 34.15226338, 34.15226372, 34.15226407,
    34.15226442, 34.15226477, 34.15226511, 34.15226546, 34.15226581,
    34.15226616, 34.15226651, 34.15226685, 34.15226720, 34.15226755,
    34.15226790, 34.15226825, 34.15226860, 34.15226895, 34.15226930,
    34.15226965, 34.15227000, 34.15227035, 34.15227070, 34.15227105,
    34.15227141, 34.15227176, 34.15227211, 34.15227246, 34.15227281,
    34.15227317, 34.15227352, 34.15227387, 34.15227422, 34.15227458,
    34.15227493, 34.15227528, 34.15227564, 34.15227599, 34.15227634,
    34.15227670, 34.15227705, 34.15227740, 34.15227775, 34.15227811
]

y_coords = [
    -77.86697539, -77.86697515, -77.86697492, -77.86697468, -77.86697445,
    -77.86697421, -77.86697397, -77.86697373, -77.86697348, -77.86697324,
    -77.86697300, -77.86697275, -77.86697250, -77.86697225, -77.86697200,
    -77.86697175, -77.86697150, -77.86697124, -77.86697099, -77.86697073,
    -77.86697047, -77.86697021, -77.86696995, -77.86696968, -77.86696942,
    -77.86696915, -77.86696888, -77.86696861, -77.86696834, -77.86696807,
    -77.86696780, -77.86696752, -77.86696725, -77.86696697, -77.86696669,
    -77.86696641, -77.86696613, -77.86696585, -77.86696556, -77.86696528,
    -77.86696499, -77.86696470, -77.86696441, -77.86696412, -77.86696383,
    -77.86696353, -77.86696324, -77.86696294, -77.86696264, -77.86696234,
    -77.86696204, -77.86696174, -77.86696143, -77.86696112, -77.86696082,
    -77.86696051, -77.86696020, -77.86695989, -77.86695957, -77.86695926,
    -77.86695894, -77.86695863, -77.86695831, -77.86695799, -77.86695767,
    -77.86695735, -77.86695702, -77.86695670, -77.86695637, -77.86695605,
    -77.86695572, -77.86695539, -77.86695506, -77.86695472, -77.86695439,
    -77.86695405, -77.86695372, -77.86695338, -77.86695304, -77.86695270,
    -77.86695236, -77.86695202, -77.86695167, -77.86695133, -77.86695098,
    -77.86695063, -77.86695029, -77.86694993, -77.86694958, -77.86694923,
    -77.86694887, -77.86694851, -77.86694815, -77.86694780, -77.86694744,
    -77.86694708, -77.86694672, -77.86694636, -77.86694599, -77.86694563
]

# Make sure distances has the same length as x_coords and y_coords
distances = [
    12.736, 12.736, 12.737, 12.737, 12.737,
    12.738, 12.738, 12.739, 12.739, 12.74,
    12.74, 12.741, 12.741, 12.741, 12.742,
    12.742, 12.743, 12.743, 12.744, 12.744,
    12.745, 12.745, 12.746, 12.746, 12.747,
    12.747, 12.748, 12.748, 12.749, 12.749,
    12.75, 12.75, 12.751, 12.751, 12.752,
    12.752, 12.753, 12.753, 12.754, 12.755,
    12.755, 12.756, 12.756, 12.757, 12.757,
    12.758, 12.759, 12.759, 12.76, 12.76,
    12.761, 12.762, 12.762, 12.763, 12.764,
    12.764, 12.765, 12.766, 12.766, 12.767,
    12.768, 12.769, 12.769, 12.77, 12.771,
    12.771, 12.772, 12.773, 12.773, 12.774,
    12.775, 12.775, 12.776, 12.777, 12.777,
    12.778, 12.779, 12.779, 12.78, 12.78
]

# Ensure all arrays have the same length
min_length = min(len(x_coords), len(y_coords), len(distances))
x_coords = x_coords[:min_length]
y_coords = y_coords[:min_length]
distances = distances[:min_length]

# Create iterations array with matching length
iterations = list(range(start_iter, start_iter + min_length))

# Create the main plot
plt.figure(figsize=(12, 10))

# Plot the path with color gradient to show direction
plt.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='viridis', 
           s=30, zorder=3, alpha=0.7)
plt.plot(x_coords, y_coords, 'b-', linewidth=1.5, alpha=0.5)

# Mark start and end points
plt.scatter(x_coords[0], y_coords[0], color='green', s=100, marker='o', label='Start', zorder=5)
plt.scatter(x_coords[-1], y_coords[-1], color='red', s=100, marker='o', label='End', zorder=5)

# Add annotations for clarity
plt.annotate(f'Start (iter {iterations[0]})', (x_coords[0], y_coords[0]), xytext=(10, 10), 
            textcoords='offset points', fontsize=12)
plt.annotate(f'End (iter {iterations[-1]})', (x_coords[-1], y_coords[-1]), xytext=(10, -10), 
            textcoords='offset points', fontsize=12)

# Add more context - label every 20th point
for i in range(0, len(x_coords), 20):
    if i < len(iterations):
        plt.annotate(f'{iterations[i]}', (x_coords[i], y_coords[i]), 
                    xytext=(5, 5), textcoords='offset points', fontsize=8)

plt.title('Rover Path', fontsize=14)
plt.xlabel('X Coordinate (Latitude)', fontsize=12)
plt.ylabel('Y Coordinate (Longitude)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(fontsize=12)

# Create a smaller inset to show the overall trend
ax_inset = plt.axes([0.6, 0.2, 0.3, 0.3])
ax_inset.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='viridis', s=20)
ax_inset.plot(x_coords, y_coords, 'b-', linewidth=1)
ax_inset.set_title('Path Overview', fontsize=10)
ax_inset.grid(True, linestyle='--', alpha=0.5)

# Add a second plot showing distance to target over iterations
plt.figure(figsize=(12, 6))
plt.plot(iterations, distances, 'r-', linewidth=2)
plt.scatter(iterations, distances, c=distances, cmap='plasma', s=30)
plt.title('Distance to Target vs. Iteration', fontsize=14)
plt.xlabel('Iteration', fontsize=12)
plt.ylabel('Distance to Target (meters)', fontsize=12)
plt.grid(True, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()