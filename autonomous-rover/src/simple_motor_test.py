#!/usr/bin/env python3

import time
import logging
import curses
from hardware.motor_controller import get_motor_controller

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("simple_test.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def main_function(stdscr):
    logger.info("Simple motor test starting")
    print("Simple motor test running")
    
    try:
        # Initialize curses
        if stdscr:
            stdscr.clear()
            stdscr.addstr(0, 0, "Simple Motor Test")
            stdscr.refresh()
        
        # Initialize motor controller
        logger.info("Initializing motor controller...")
        motor = get_motor_controller()
        logger.info("Motor controller initialized")
        
        # Set initial motor speed to 0
        motor.set_speed(0.0)
        logger.info("Set initial motor speed to 0")
        
        # Gradually increase to 0.3 PWM
        logger.info("Increasing speed to 0.3 PWM")
        for i in range(30):
            pwm = i / 100.0
            motor.set_speed(pwm)
            logger.info(f"PWM set to {pwm:.2f}")
            if stdscr:
                stdscr.addstr(1, 0, f"Current PWM: {pwm:.2f}")
                stdscr.refresh()
            time.sleep(0.1)
        
        # Hold at 0.3 PWM for 2 seconds
        logger.info("Holding at 0.3 PWM for 2 seconds")
        if stdscr:
            stdscr.addstr(2, 0, "Holding at 0.3 PWM")
            stdscr.refresh()
        time.sleep(2)
        
        # Gradually decrease to 0.0 PWM
        logger.info("Decreasing speed to 0.0 PWM")
        for i in range(30, -1, -1):
            pwm = i / 100.0
            motor.set_speed(pwm)
            logger.info(f"PWM set to {pwm:.2f}")
            if stdscr:
                stdscr.addstr(1, 0, f"Current PWM: {pwm:.2f}")
                stdscr.refresh()
            time.sleep(0.1)
        
        # Stop the motor
        logger.info("Stopping motor")
        motor.set_speed(0.0)
        
        if stdscr:
            stdscr.addstr(3, 0, "Test complete. Press any key to exit.")
            stdscr.refresh()
            stdscr.nodelay(False)
            stdscr.getch()
        
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        if stdscr:
            try:
                stdscr.addstr(0, 0, f"ERROR: {str(e)}")
                stdscr.refresh()
                time.sleep(2)
            except:
                pass
    finally:
        logger.info("Test complete")

# First version without curses
if __name__ == "__main__":
    try:
        logger.info("Starting without curses")
        main_function(None)
    except Exception as e:
        logger.error(f"Error in main: {e}")
        import traceback
        logger.error(traceback.format_exc())

# If the above works, uncomment this to test with curses
"""
if __name__ == "__main__":
    try:
        logger.info("Starting with curses wrapper")
        curses.wrapper(main_function)
    except Exception as e:
        logger.error(f"Error in curses wrapper: {e}")
        import traceback
        logger.error(traceback.format_exc())
"""