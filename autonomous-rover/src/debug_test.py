#!/usr/bin/env python3

import time
import logging
import sys

# Configure basic logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("debug_test.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def main():
    logger.info("Debug test starting")
    print("Debug test running - you should see this in the terminal")
    
    try:
        # Test imports
        logger.info("Testing imports...")
        import curses
        logger.info("Curses import successful")
        
        from hardware.motor_controller import get_motor_controller
        logger.info("Motor controller import successful")
        
        import requests
        logger.info("Requests import successful")
        
        # Try to initialize motor controller
        logger.info("Trying to initialize motor controller...")
        motor = get_motor_controller()
        if motor:
            logger.info(f"Motor controller initialized: {motor}")
        else:
            logger.error("Motor controller returned None")
        
        # Try to get GPS data
        logger.info("Trying to get GPS data...")
        try:
            response = requests.get('http://localhost:5001/api/gps', timeout=1.0)
            logger.info(f"GPS API response: status={response.status_code}")
        except Exception as e:
            logger.error(f"GPS API request failed: {e}")
        
        # Wait a bit to see logs
        logger.info("Test complete. Waiting 2 seconds...")
        time.sleep(2)
    
    except Exception as e:
        logger.error(f"Error during testing: {e}")
        import traceback
        logger.error(traceback.format_exc())

if __name__ == "__main__":
    main()
    print("Script completed - check debug_test.log for details")