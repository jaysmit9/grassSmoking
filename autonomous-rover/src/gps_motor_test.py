#!/usr/bin/env python3

import time
import logging
import curses
import requests
import json
from hardware.motor_controller import get_motor_controller

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("gps_test.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

def get_gps_data():
    """Fetch GPS data from the service running on localhost:5001."""
    try:
        response = requests.get('http://localhost:5001/api/gps', timeout=0.5)
        if response.status_code == 200:
            data = response.json()
            return data
    except Exception as e:
        logger.debug(f"Failed to get GPS data: {e}")
    return None

def format_speed(speed):
    """Format speed value for display."""
    if speed is None:
        return "N/A"
    if isinstance(speed, (int, float)):
        return f"{speed:.2f} m/s"
    return str(speed)

def gps_motor_test(stdscr=None):
    logger.info("GPS motor test starting")
    print("GPS motor test running")
    
    try:
        # Initialize curses if provided
        if stdscr:
            curses.noecho()
            curses.cbreak()
            stdscr.keypad(True)
            stdscr.nodelay(True)  # Non-blocking input
            stdscr.clear()
            stdscr.addstr(0, 0, "GPS Motor Test")
            stdscr.refresh()
        
        # Initialize motor controller
        logger.info("Initializing motor controller...")
        motor = get_motor_controller()
        logger.info("Motor controller initialized")
        
        # Set initial motor speed to 0
        motor.set_speed(0.0)
        logger.info("Set initial motor speed to 0")
        
        # Get initial GPS data
        logger.info("Fetching initial GPS data...")
        gps_data = get_gps_data()
        if gps_data:
            logger.info("GPS data received")
            # Try to extract speed
            front_gps = gps_data.get('devices', {}).get('Front GPS', {})
            speed = front_gps.get('speed', 'N/A')
            logger.info(f"Current GPS speed: {format_speed(speed)}")
        else:
            logger.warning("Could not get GPS data")
        
        # Display GPS info if using curses
        if stdscr:
            speed_str = format_speed(speed) if 'speed' in locals() else "N/A"
            stdscr.addstr(1, 0, f"GPS Speed: {speed_str}")
            stdscr.addstr(2, 0, "Press SPACE to start motor test")
            stdscr.addstr(3, 0, "Press Q to quit")
            stdscr.refresh()
            
            # Wait for space bar
            key = None
            while key != ord(' ') and key != ord('q'):
                key = stdscr.getch()
                time.sleep(0.1)
                
                # Update GPS data while waiting
                gps_data = get_gps_data()
                if gps_data:
                    front_gps = gps_data.get('devices', {}).get('Front GPS', {})
                    speed = front_gps.get('speed', 'N/A')
                    speed_str = format_speed(speed)
                    stdscr.addstr(1, 0, f"GPS Speed: {speed_str}")
                    stdscr.refresh()
            
            if key == ord('q'):
                logger.info("User quit before starting test")
                return
        
        # Gradually increase to 0.3 PWM
        logger.info("Increasing speed to 0.3 PWM")
        for i in range(30):
            pwm = i / 100.0
            motor.set_speed(pwm)
            logger.info(f"PWM set to {pwm:.2f}")
            
            # Update GPS data and display
            gps_data = get_gps_data()
            if gps_data:
                front_gps = gps_data.get('devices', {}).get('Front GPS', {})
                speed = front_gps.get('speed', 'N/A')
                speed_str = format_speed(speed)
                logger.info(f"GPS Speed: {speed_str}")
                
                if stdscr:
                    stdscr.addstr(1, 0, f"GPS Speed: {speed_str}")
                    stdscr.addstr(4, 0, f"Current PWM: {pwm:.2f}")
                    stdscr.refresh()
            
            # Check for quit key if using curses
            if stdscr:
                key = stdscr.getch()
                if key == ord('q'):
                    logger.info("User quit during speed increase")
                    motor.set_speed(0.0)
                    return
            
            time.sleep(0.1)
        
        # Hold at 0.3 PWM for 2 seconds
        logger.info("Holding at 0.3 PWM for 2 seconds")
        if stdscr:
            stdscr.addstr(5, 0, "Holding at 0.3 PWM")
            stdscr.refresh()
            
        start_time = time.time()
        while time.time() - start_time < 2:
            # Update GPS data and display
            gps_data = get_gps_data()
            if gps_data:
                front_gps = gps_data.get('devices', {}).get('Front GPS', {})
                speed = front_gps.get('speed', 'N/A')
                speed_str = format_speed(speed)
                
                if stdscr:
                    stdscr.addstr(1, 0, f"GPS Speed: {speed_str}")
                    stdscr.refresh()
            
            # Check for quit key if using curses
            if stdscr:
                key = stdscr.getch()
                if key == ord('q'):
                    logger.info("User quit during hold")
                    motor.set_speed(0.0)
                    return
            
            time.sleep(0.1)
        
        # Gradually decrease to 0.0 PWM
        logger.info("Decreasing speed to 0.0 PWM")
        for i in range(30, -1, -1):
            pwm = i / 100.0
            motor.set_speed(pwm)
            logger.info(f"PWM set to {pwm:.2f}")
            
            # Update GPS data and display
            gps_data = get_gps_data()
            if gps_data:
                front_gps = gps_data.get('devices', {}).get('Front GPS', {})
                speed = front_gps.get('speed', 'N/A')
                speed_str = format_speed(speed)
                
                if stdscr:
                    stdscr.addstr(1, 0, f"GPS Speed: {speed_str}")
                    stdscr.addstr(4, 0, f"Current PWM: {pwm:.2f}")
                    stdscr.refresh()
            
            # Check for quit key if using curses
            if stdscr:
                key = stdscr.getch()
                if key == ord('q'):
                    logger.info("User quit during deceleration")
                    motor.set_speed(0.0)
                    return
            
            time.sleep(0.1)
        
        # Stop the motor
        logger.info("Stopping motor")
        motor.set_speed(0.0)
        
        if stdscr:
            stdscr.addstr(6, 0, "Test complete. Press any key to exit.")
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
        # Ensure motor is stopped
        if 'motor' in locals():
            motor.set_speed(0.0)
        
        # Clean up curses
        if stdscr:
            try:
                curses.nocbreak()
                stdscr.keypad(False)
                curses.echo()
            except:
                pass
            
        logger.info("Test complete")

# Try both with and without curses
if __name__ == "__main__":
    try:
        # First run without curses
        print("Starting test without curses...")
        logger.info("Starting without curses")
        gps_motor_test()
        
        # Then run with curses
        print("Starting test with curses...")
        logger.info("Starting with curses")
        curses.wrapper(gps_motor_test)
    except Exception as e:
        logger.error(f"Error in main: {e}")
        import traceback
        logger.error(traceback.format_exc())