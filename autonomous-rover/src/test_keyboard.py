#!/usr/bin/env python3

import time
import signal
import sys
import logging
import os

# Add the current directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from input.keyboard_handler import get_keyboard_handler
from hardware.motor_controller import get_motor_controller

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def signal_handler(sig, frame):
    """Handle Ctrl+C to exit gracefully."""
    logger.info("Stopping the rover...")
    get_motor_controller().stop()
    keyboard_handler.stop()
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize motor controller
    motor = get_motor_controller()
    
    # Start keyboard handler
    keyboard_handler = get_keyboard_handler()
    keyboard_handler.start()
    
    logger.info("Rover ready for keyboard control (arrow keys)")
    logger.info("Press Ctrl+C to exit")
    
    # Keep the main thread alive
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        signal_handler(None, None)
