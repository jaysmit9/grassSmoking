#!/bin/bash

# Go to the script directory
cd "$(dirname "$0")"
echo "Running from directory: $(pwd)"

# Kill any existing instances that might be running
echo "Checking for existing processes..."
pkill -f "python3 grid_distance_detection.py" || true

# Run the detection script with real mode
echo "Starting detector with integrated API..."
python3 grid_distance_detection.py --real

# The script will run in the foreground so you can see the logs
# Press Ctrl+C to stop
