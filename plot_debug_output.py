import matplotlib.pyplot as plt
import json

# Load debug output data from file
with open('debug_output.txt', 'r') as f:
    lines = f.readlines()

# Extract x_history and y_history from the file
x_history = []
y_history = []
for line in lines:
    if line.startswith("Iteration"):
        parts = line.split(", ")
        x = float(parts[1].split("=")[1])
        y = float(parts[2].split("=")[1])
        x_history.append(x)
        y_history.append(y)

# Plot the results
plt.figure()
plt.plot(x_history, y_history, 'b-', label='Trajectory')
plt.xlabel('Latitude')
plt.ylabel('Longitude')
plt.legend()
plt.title('Stanley Controller Path Tracking from Debug Output')
plt.grid(True)
plt.show()