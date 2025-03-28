from flask import Flask, jsonify
import threading
import time
import json
import sys
import os

# Add the parent directory to sys.path so we can import our module
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

# Import our detection module
import grid_distance_detection as gdd

app = Flask(__name__)

@app.route('/')
def home():
    """Root endpoint - simple welcome message"""
    return "RealSense Detection API - Use /detection to access detection data"

@app.route('/detection', methods=['GET'])
def get_detection():
    """API endpoint to get the latest detection data"""
    detection_data = gdd.get_latest_detection()
    
    if detection_data is None:
        return jsonify({"error": "No detection data available yet"}), 404
    
    return jsonify(detection_data)

@app.route('/health', methods=['GET'])
def health_check():
    """Simple health check endpoint"""
    return jsonify({"status": "ok", "timestamp": time.time()})

# Prevent the main detection script from running when imported
# This block only runs if test_api.py is executed directly
if __name__ == '__main__':
    print("Starting Flask API server on port 5000...")
    # Run Flask without debug mode for production use
    app.run(host='0.0.0.0', port=5000)
