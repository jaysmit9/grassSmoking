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
        logging.FileHandler("speed_control.log"),
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
    except (requests.RequestException, json.JSONDecodeError) as e:
        logger.debug(f"Failed to get GPS data: {e}")
    return None

def get_current_speed(gps_data):
    """Extract speed in m/s from GPS data."""
    if not gps_data:
        return 0.0
    
    try:
        # Try to get speed from front GPS
        front_gps = gps_data.get('devices', {}).get('Front GPS', {})
        speed = front_gps.get('speed', 0.0)
        if speed is None:
            speed = 0.0
        return float(speed)
    except (TypeError, ValueError):
        return 0.0

def display_info(stdscr, gps_data, pwm_value, target_speed):
    """Display GPS and control information on the screen."""
    if gps_data:
        # Clear previous data
        for i in range(10):
            stdscr.addstr(i, 0, " " * 60)
        
        # Extract values from the nested JSON structure
        # Dual GPS bearing (heading)
        heading = gps_data.get('dual_gps', {}).get('bearing', 'N/A')
        heading_str = f"{heading:>6.1f}°" if isinstance(heading, (int, float)) else "N/A"
        
        # GPS speeds
        front_gps = gps_data.get('devices', {}).get('Front GPS', {})
        speed = front_gps.get('speed', 0.0)
        speed_str = f"{speed:>6.2f} m/s" if isinstance(speed, (int, float)) else "N/A"
        
        # Get coordinates from front GPS
        lat = front_gps.get('lat', 'N/A')
        lat_str = f"{lat:.6f}" if isinstance(lat, (int, float)) else "N/A"
        
        lon = front_gps.get('lon', 'N/A')
        lon_str = f"{lon:.6f}" if isinstance(lon, (int, float)) else "N/A"
        
        # Display speed control info
        stdscr.addstr(0, 0, f"GPS Speed: {speed_str}")
        stdscr.addstr(1, 0, f"Target Speed: {target_speed:.2f} m/s")
        stdscr.addstr(2, 0, f"PWM Value: {pwm_value:.2f}")
        
        # Speed error (how far from target)
        error = 0.0
        if isinstance(speed, (int, float)):
            error = target_speed - speed
        stdscr.addstr(3, 0, f"Speed Error: {error:.2f} m/s")
        
        # GPS info
        stdscr.addstr(5, 0, f"Heading: {heading_str}")
        stdscr.addstr(6, 0, f"Location: {lat_str}, {lon_str}")
        
        # Control instructions
        stdscr.addstr(8, 0, "SPACE: Emergency Stop | Q: Quit | +/-: Adjust Target")
    else:
        stdscr.addstr(0, 0, "GPS Data: Not available")
        stdscr.addstr(1, 0, f"PWM Value: {pwm_value:.2f}")
        stdscr.addstr(2, 0, f"Target Speed: {target_speed:.2f} m/s")
    
    stdscr.refresh()

def speed_controller(stdscr, reverse=False, target_speed=0.5):
    logger.info("Starting GPS speed controller")
    logger.info(f"Reverse mode: {reverse}, Target speed: {target_speed} m/s")
    
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
        logger.info("Initializing motor controller...")
        motor = get_motor_controller(max_speed=0.8)  # Allow speeds up to 0.8
        if motor is None:
            logger.error("Failed to initialize motor controller!")
            stdscr.addstr(0, 0, "ERROR: Failed to initialize motor controller!")
            stdscr.refresh()
            time.sleep(3)
            return
        
        logger.info("Motor controller initialized successfully")
        
        # Set initial PWM to 0
        motor.set_speed(0.0)
        
        # PID controller parameters
        kp = 0.5  # Proportional gain
        ki = 0.05  # Integral gain
        integral = 0.0
        
        # Initial PWM value
        pwm_value = 0.0
        
        # Speed direction
        direction = -1.0 if reverse else 1.0
        
        # Display instructions
        stdscr.addstr(height-2, 0, "Press SPACE to stop | Q to quit | +/- to adjust target speed")
        stdscr.refresh()
        
        # Get initial GPS data
        logger.info("Fetching initial GPS data")
        gps_data = get_gps_data()
        display_info(stdscr, gps_data, pwm_value, target_speed)
        
        # Wait for 2 seconds to allow user to see the initial display
        stdscr.addstr(height-4, 0, "Starting in 2 seconds...")
        stdscr.refresh()
        time.sleep(2)
        
        # Main control loop
        running = True
        last_time = time.time()
        while running:
            # Get current time
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Check for key presses
            key = stdscr.getch()
            if key == ord(' '):
                logger.info("Space bar pressed! Stopping motors immediately.")
                motor.set_speed(0.0)
                pwm_value = 0.0
                stdscr.addstr(height-4, 0, "EMERGENCY STOP ACTIVATED               ")
                stdscr.refresh()
                time.sleep(1)
                running = False
                continue
            elif key == ord('q') or key == ord('Q'):
                logger.info("Quit command received.")
                motor.set_speed(0.0)
                running = False
                continue
            elif key == ord('+') or key == ord('='):
                target_speed += 0.1
                logger.info(f"Target speed increased to {target_speed:.1f} m/s")
            elif key == ord('-') or key == ord('_'):
                target_speed = max(0.1, target_speed - 0.1)
                logger.info(f"Target speed decreased to {target_speed:.1f} m/s")
            
            # Get GPS data
            gps_data = get_gps_data()
            current_speed = get_current_speed(gps_data)
            
            # Calculate error
            error = target_speed - current_speed
            
            # Update integral term with anti-windup
            integral += error * dt
            integral = max(-1.0, min(1.0, integral))  # Clamp integral term
            
            # Calculate control signal (PWM adjustment)
            control = kp * error + ki * integral
            
            # Update PWM value
            pwm_value += control * 0.05  # Scale control effect for smoother changes
            pwm_value = max(-0.7, min(0.7, pwm_value))  # Clamp PWM value
            
            # Apply PWM value with direction
            motor_pwm = direction * pwm_value
            motor.set_speed(motor_pwm)
            
            # Display status
            display_info(stdscr, gps_data, motor_pwm, target_speed)
            
            # Add control loop status
            stdscr.addstr(4, 0, f"Control: P={kp*error:.3f} I={ki*integral:.3f}")
            
            # Sleep for a short time
            time.sleep(0.1)  # 10Hz control loop
        
        # Stop the motors
        logger.info("Stopping motors")
        motor.set_speed(0.0)
        
        # Final message
        stdscr.addstr(height-4, 0, "Controller stopped. Press any key to exit.")
        stdscr.refresh()
        stdscr.nodelay(False)  # Wait for keypress
        stdscr.getch()
    
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
        logger.info("Speed controller completed")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="GPS-based speed controller")
    parser.add_argument("-r", "--reverse", action="store_true", help="Run in reverse mode")
    parser.add_argument("-s", "--speed", type=float, default=0.5, help="Target speed in m/s")
    args = parser.parse_args()

    logger.info("GPS speed controller starting")
    
    try:
        # Run the controller with curses wrapper
        curses.wrapper(speed_controller, reverse=args.reverse, target_speed=args.speed)
    except Exception as e:
        logger.error(f"Failed to run speed controller: {e}")
        logger.error(traceback.format_exc())