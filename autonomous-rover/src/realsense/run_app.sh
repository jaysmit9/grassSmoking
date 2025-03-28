#!/bin/bash

# Go to the script directory
cd "$(dirname "$0")"
echo "Working directory: $(pwd)"

# Check if we have the required files
echo "Checking setup..."
python3 check_setup.py

if [ $? -ne 0 ]; then
    echo "Setup check failed. Please fix the issues and try again."
    exit 1
fi

# Create a marker file to indicate we're running
marker="/tmp/realsense_api_running.lock"
echo "$(date)" > "$marker"
echo "Created marker at $marker"

# Kill any existing instances that might be running
echo "Checking for existing processes..."
pkill -f "python3 combined_app.py" || true
echo "Killed existing processes if any"

# Run the combined app with output to a log file
log_file="./logs/run_app_$(date +%Y%m%d_%H%M%S).log"
mkdir -p ./logs

echo "Starting combined app, logging to $log_file..."
python3 -u combined_app.py > "$log_file" 2>&1 &

pid=$!
echo "Combined app started with PID: $pid"

# Wait a bit to see if it crashes immediately
sleep 3

if ps -p $pid > /dev/null; then
    echo "App is running with PID $pid. Access the API at: http://localhost:5000/detection"
    echo "View logs in real-time with: tail -f $log_file"
    echo "Press Ctrl+C to stop following logs (app will continue running)"
    tail -f "$log_file"
else
    echo "Error: App failed to start. Check logs at $log_file"
    cat "$log_file"
    exit 1
fi
