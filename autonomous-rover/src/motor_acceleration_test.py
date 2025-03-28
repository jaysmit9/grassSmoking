#!/usr/bin/env python3

import time
import logging
import curses
import argparse
import requests
import json
import sys
import traceback
from hardware.motor_controller import get_motor_controller

# Configure logging to file AND console
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("motor_test.log"),
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
            logger.debug(f"Raw GPS data: {json.dumps(data)[:100]}...")  # Log first 100 chars for debugging
            return data
    except (requests.RequestException, json.JSONDecodeError) as e:
        logger.debug(f"Failed to get GPS data: {e}")
    return None

def display_gps_info(stdscr, gps_data):
    """Display GPS information on the screen."""
    if gps_data:
        # Clear previous data
        for i in range(5):
            stdscr.addstr(i, 0, " " * 50)
        
        # Extract values from the nested JSON structure
        # Dual GPS bearing (heading)
        heading = gps_data.get('dual_gps', {}).get('bearing', 'N/A')
        heading_str = f"{heading:>6.1f}°" if isinstance(heading, (int, float)) else f"{'N/A':>6s} "
        
        # Front GPS heading (track)
        front_gps = gps_data.get('devices', {}).get('Front GPS', {})
        gps1_heading = front_gps.get('track', 'N/A')
        gps1_str = f"{gps1_heading:>6.1f}°" if isinstance(gps1_heading, (int, float)) else f"{'N/A':>6s} "
        
        # Rear GPS heading (track)
        rear_gps = gps_data.get('devices', {}).get('Rear GPS', {})
        gps2_heading = rear_gps.get('track', 'N/A')
        gps2_str = f"{gps2_heading:>6.1f}°" if isinstance(gps2_heading, (int, float)) else f"{'N/A':>6s} "
        
        # Get speed (from front GPS if available, otherwise N/A)
        speed = front_gps.get('speed', 'N/A')
        speed_str = f"{speed:>6.2f} m/s" if isinstance(speed, (int, float)) else f"{'N/A':>6s} m/s"
        
        # Get coordinates from front GPS
        lat = front_gps.get('lat', 'N/A')
        lat_str = f"{lat:.6f}" if isinstance(lat, (int, float)) else "N/A"
        
        lon = front_gps.get('lon', 'N/A')
        lon_str = f"{lon:.6f}" if isinstance(lon, (int, float)) else "N/A"
        
        # Get satellite counts
        front_sats = front_gps.get('satellites_used', 0)
        rear_sats = rear_gps.get('satellites_used', 0)
        total_sats = front_sats + rear_sats if isinstance(front_sats, int) and isinstance(rear_sats, int) else 'N/A'
        
        # RTK mode information
        front_rtk = front_gps.get('rtk_mode', 'N/A')
        rear_rtk = rear_gps.get('rtk_mode', 'N/A')
        
        # Display GPS data
        stdscr.addstr(0, 0, f"Dual GPS Heading: {heading_str}")
        stdscr.addstr(1, 0, f"GPS1: {gps1_str} ({front_rtk})  GPS2: {gps2_str} ({rear_rtk})")
        stdscr.addstr(2, 0, f"Speed: {speed_str}")
        stdscr.addstr(3, 0, f"Lat: {lat_str}  Lon: {lon_str}")
        stdscr.addstr(4, 0, f"Satellites: {front_sats}/{rear_sats} (Total: {total_sats})")
    else:
        stdscr.addstr(0, 0, "GPS Data: Not available")
    
    stdscr.refresh()

def motor_acceleration_test(stdscr, reverse=False):
    logger.info("Starting motor acceleration test")
    logger.info(f"Reverse mode: {reverse}")
    
    try:
        # Initialize curses
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)  # Non-blocking input
        
        # Clear screen
        stdscr.clear()
        
        # Set up display area
        height, width = stdscr.getmaxyx()
        logger.info(f"Terminal size: {width}x{height}")
        
        # Initialize the motor controller
        logger.info("Initializing motor controller with higher max_speed...")
        motor = get_motor_controller(max_speed=0.8)  # Allow speeds up to 0.8
        if motor is None:
            logger.error("Failed to initialize motor controller!")
            stdscr.addstr(0, 0, "ERROR: Failed to initialize motor controller!")
            stdscr.refresh()
            time.sleep(3)
            return
        
        logger.info("Motor controller initialized successfully")
        
        # Set initial PWM to 0 for both motors
        logger.info("Setting initial motor speed to 0")
        motor.set_speed(0.0)
        
        # Display instructions
        stdscr.addstr(height-2, 0, "Press SPACE to stop motors immediately | Press CTRL+C to exit")
        stdscr.refresh()
        
        # Get initial GPS data
        logger.info("Fetching initial GPS data")
        gps_data = get_gps_data()
        display_gps_info(stdscr, gps_data)
        
        # DEBUGGING - add a pause to see if display is working
        stdscr.addstr(8, 0, "Starting in 3 seconds...")
        stdscr.refresh()
        time.sleep(3)
        
        if reverse:
            # Accelerate to -0.4 PWM over 2 seconds
            logger.info("Accelerating to -0.4 PWM")
            target_speed = -0.7
            current_speed = 0.0
            increment = 0.01
            
            stdscr.addstr(8, 0, "Beginning reverse acceleration")
            stdscr.refresh()
            
            while current_speed > target_speed:
                # Update display
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM (Target: {target_speed:.2f})")
                stdscr.addstr(7, 0, "Status: Accelerating to reverse")
                stdscr.refresh()
                
                # Check for space bar
                key = stdscr.getch()
                if key == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update speed
                current_speed = max(current_speed - increment, target_speed)
                logger.debug(f"Setting motor speed to {current_speed}")
                motor.set_speed(current_speed)
                
                # Update GPS data every few iterations
                if round(current_speed * 100) % 5 == 0:
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)  # 20Hz update rate
            
            # Maintain -0.4 PWM for 2 seconds
            logger.info("Maintaining -0.4 PWM for 2 seconds")
            start_time = time.time()
            while time.time() - start_time < 2:
                # Update display
                elapsed = time.time() - start_time
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM")
                stdscr.addstr(7, 0, f"Status: Maintaining reverse ({elapsed:.1f}/2.0s)")
                stdscr.refresh()
                
                # Check for space bar
                if stdscr.getch() == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update GPS data
                if round(elapsed * 5) % 2 == 0:  # Update every ~0.4 seconds
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)
            
            # Decelerate to 0 PWM over 2 seconds
            logger.info("Decelerating to 0 PWM")
            while current_speed < 0.0:
                # Update display
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM (Target: 0.00)")
                stdscr.addstr(7, 0, "Status: Decelerating to stop")
                stdscr.refresh()
                
                # Check for space bar
                if stdscr.getch() == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update speed
                current_speed = min(current_speed + increment, 0.0)
                motor.set_speed(current_speed)
                
                # Update GPS data
                if round(current_speed * 100) % 5 == 0:
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)  # 20Hz update rate
            
        else:
            # Accelerate to 0.4 PWM over 2 seconds
            logger.info("Accelerating to 0.4 PWM")
            target_speed = 0.7
            current_speed = 0.0
            increment = 0.01
            
            stdscr.addstr(8, 0, "Beginning forward acceleration")
            stdscr.refresh()
            
            while current_speed < target_speed:
                # Update display
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM (Target: {target_speed:.2f})")
                stdscr.addstr(7, 0, "Status: Accelerating forward")
                stdscr.refresh()
                
                # Check for space bar
                key = stdscr.getch()
                if key == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update speed
                current_speed = min(current_speed + increment, target_speed)
                logger.debug(f"Setting motor speed to {current_speed}")
                motor.set_speed(current_speed)
                
                # Update GPS data every few iterations
                if round(current_speed * 100) % 5 == 0:
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)  # 20Hz update rate
            
            # Maintain 0.4 PWM for 2 seconds
            logger.info("Maintaining 0.4 PWM for 2 seconds")
            start_time = time.time()
            while time.time() - start_time < 2:
                # Update display
                elapsed = time.time() - start_time
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM")
                stdscr.addstr(7, 0, f"Status: Maintaining forward ({elapsed:.1f}/2.0s)")
                stdscr.refresh()
                
                # Check for space bar
                if stdscr.getch() == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update GPS data
                if round(elapsed * 5) % 2 == 0:  # Update every ~0.4 seconds
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)
            
            # Decelerate to 0 PWM over 2 seconds
            logger.info("Decelerating to 0 PWM")
            while current_speed > 0.0:
                # Update display
                stdscr.addstr(6, 0, f"Current Speed: {current_speed:.2f} PWM (Target: 0.00)")
                stdscr.addstr(7, 0, "Status: Decelerating to stop")
                stdscr.refresh()
                
                # Check for space bar
                if stdscr.getch() == ord(' '):
                    logger.info("Space bar pressed! Stopping motors immediately.")
                    motor.set_speed(0.0)
                    return
                
                # Update speed
                current_speed = max(current_speed - increment, 0.0)
                motor.set_speed(current_speed)
                
                # Update GPS data
                if round(current_speed * 100) % 5 == 0:
                    gps_data = get_gps_data()
                    display_gps_info(stdscr, gps_data)
                
                time.sleep(0.05)  # 20Hz update rate
        
        # Stop the motors
        logger.info("Stopping motors")
        motor.set_speed(0.0)
    
    except KeyboardInterrupt:
        logger.info("Interrupted! Stopping motors")
        if 'motor' in locals():
            motor.set_speed(0.0)
    
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        logger.error(traceback.format_exc())
        if 'motor' in locals():
            motor.set_speed(0.0)
        if 'stdscr' in locals():
            try:
                stdscr.addstr(0, 0, f"ERROR: {str(e)}")
                stdscr.refresh()
                time.sleep(3)
            except:
                pass
    
    finally:
        # Restore terminal settings
        logger.info("Cleaning up and exiting")
        try:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()
        except:
            pass
        logger.info("Motor acceleration test completed")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Motor acceleration test script")
    parser.add_argument("-r", "--reverse", action="store_true", help="Run the test in reverse mode")
    args = parser.parse_args()

    logger.info("Motor acceleration test starting")
    
    try:
        # Run the test with curses wrapper
        curses.wrapper(motor_acceleration_test, reverse=args.reverse)
    except Exception as e:
        logger.error(f"Failed to run motor test: {e}")
        logger.error(traceback.format_exc())