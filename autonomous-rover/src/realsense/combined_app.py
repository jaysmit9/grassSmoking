import threading
import time
import subprocess
import os
import sys
import importlib
import logging
import signal
import json
import traceback  # Add missing import for traceback

# Configure logging with timestamp to both file and console
log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, f"combined_app_{time.strftime('%Y%m%d_%H%M%S')}.log")

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        logging.StreamHandler(sys.stdout)
    ]
)
logger = logging.getLogger("combined_app")

# Print a startup message to make sure we can see this is running
print(f"Starting combined app, logging to {log_file}")
logger.info(f"Combined app started, logging to {log_file}")

# Add directory to path
current_dir = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_dir)

# Global process handle for clean shutdown
detection_process = None
detection_data = None

def run_detection():
    """Run the detection script in a separate process"""
    global detection_process
    
    logger.info("Starting detection process...")
    
    # Get the path to the detection script
    detection_script = os.path.join(current_dir, "grid_distance_detection.py")
    
    # Check if the script exists
    if not os.path.exists(detection_script):
        logger.error(f"Detection script not found at: {detection_script}")
        return
    
    try:
        # Run the detection script with real mode
        detection_process = subprocess.Popen(
            [sys.executable, detection_script, "--real"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1  # Line buffered
        )
        
        logger.info(f"Detection process started with PID: {detection_process.pid}")
        
        # Monitor stdout in a separate thread to avoid blocking
        def read_stdout():
            for line in detection_process.stdout:
                # Check if this is a detection data line
                if line.strip().startswith("Detected "):
                    logger.info(f"Detection data available")
                else:
                    logger.info(f"Detection: {line.strip()}")
        
        # Monitor stderr in a separate thread
        def read_stderr():
            for line in detection_process.stderr:
                logger.error(f"Detection error: {line.strip()}")
        
        # Start monitoring threads
        stdout_thread = threading.Thread(target=read_stdout)
        stderr_thread = threading.Thread(target=read_stderr)
        stdout_thread.daemon = True
        stderr_thread.daemon = True
        stdout_thread.start()
        stderr_thread.start()
        
        # Wait for process to complete
        detection_process.wait()
        logger.info(f"Detection process exited with code {detection_process.returncode}")
        
    except Exception as e:
        logger.error(f"Error starting detection process: {str(e)}")

# Import Flask only after setting up logging to avoid interference
from flask import Flask, jsonify

# Create Flask app with a custom logger to integrate with our logging setup
app = Flask(__name__)
app.logger.handlers = []
for handler in logging.getLogger().handlers:
    app.logger.addHandler(handler)
app.logger.setLevel(logging.INFO)

# Try to import our module later when needed
def get_detection_module():
    """Safely import the detection module"""
    try:
        import grid_distance_detection
        logger.info("Successfully imported grid_distance_detection module")
        return grid_distance_detection
    except Exception as e:
        logger.error(f"Error importing grid_distance_detection: {str(e)}")
        logger.error(traceback.format_exc())
        return None

@app.route('/')
def home():
    """Root endpoint - simple welcome message"""
    return "RealSense Detection API - Use /detection to access detection data"

@app.route('/detection', methods=['GET'])
def get_detection():
    """API endpoint to get the latest detection data"""
    try:
        gdd = get_detection_module()
        if gdd:
            detection_data = gdd.get_latest_detection()
            if detection_data:
                logger.info(f"Returning detection data with {len(detection_data.get('objects', []))} objects")
                return jsonify(detection_data)
        
        # If we get here, no data is available
        logger.warning("No detection data available")
        return jsonify({"error": "No detection data available yet"}), 404
        
    except Exception as e:
        logger.error(f"Error getting detection data: {str(e)}")
        return jsonify({"error": str(e)}), 500

@app.route('/health', methods=['GET'])
def health_check():
    """Simple health check endpoint"""
    try:
        gdd = get_detection_module()
        detection_status = "running" if detection_process and detection_process.poll() is None else "not running"
        has_data = False
        if gdd:
            has_data = gdd.get_latest_detection() is not None
        
        return jsonify({
            "status": "ok", 
            "timestamp": time.time(),
            "detection_status": detection_status,
            "has_data": has_data
        })
    except Exception as e:
        logger.error(f"Error in health check: {str(e)}")
        return jsonify({
            "status": "error",
            "error": str(e)
        }), 500

def run_flask():
    """Run the Flask API server"""
    logger.info("Starting Flask API server...")
    try:
        # Use debug=False in production
        app.run(host='0.0.0.0', port=5000, debug=False)
    except Exception as e:
        logger.error(f"Flask error: {str(e)}")

def signal_handler(sig, frame):
    """Handle Ctrl+C and other signals gracefully"""
    logger.info("Shutdown signal received, cleaning up...")
    if detection_process and detection_process.poll() is None:
        logger.info("Terminating detection process...")
        detection_process.terminate()
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logger.info("Starting combined app...")
    
    try:
        # Start detection in a separate thread
        detection_thread = threading.Thread(target=run_detection)
        detection_thread.start()
        
        # Give detection time to initialize
        logger.info("Waiting for detection to initialize...")
        time.sleep(5)
        
        # Start Flask in the main thread
        logger.info("Starting Flask server...")
        run_flask()
        
    except Exception as e:
        logger.error(f"Error in combined app: {str(e)}")
        if detection_process and detection_process.poll() is None:
            detection_process.terminate()
        sys.exit(1)
