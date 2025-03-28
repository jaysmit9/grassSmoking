#!/usr/bin/env python3

import time
import logging
import curses
import argparse
import requests
import json
from hardware.motor_controller import get_motor_controller

# Configure logging
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
    except Exception as e:
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

def format_value(value, format_str="%.2f", na_str="N/A"):
    """Format a value for display, handling None and non-numeric values."""
    if value is None:
        return na_str
    try:
        return format_str % float(value)
    except (ValueError, TypeError):
        return na_str

def speed_controller(stdscr, reverse=False, target_speed=0.5):
    """Main speed controller function."""
    logger.info(f"Starting GPS speed controller (reverse={reverse}, target={target_speed})")
    
    try:
        # Initialize curses
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)  # Non-blocking input
        curses.curs_set(0)    # Hide cursor
        
        # Clear screen
        stdscr.clear()
        
        # Get terminal dimensions
        height, width = stdscr.getmaxyx()
        
        # Initialize the motor controller
        logger.info("Initializing motor controller...")
        motor = get_motor_controller(max_speed=0.8)  # Allow higher speeds
        logger.info("Motor controller initialized")
        
        # Set initial PWM to 0
        motor.set_speed(0.0)
        current_pwm = 0.0
        last_pwm_update_time = time.time()
        logger.info("Initial motor speed set to 0")
        
        # PID controller parameters (adjusted for more responsiveness)
        kp = 0.4   # Increased for more responsiveness
        ki = 0.03  # Slightly increased
        kd = 0.05  # Keep the same
        
        # Control state variables
        integral = 0.0
        last_error = 0.0
        
        # Speed direction
        direction = -1.0 if reverse else 1.0
        
        # Controller state
        running = False
        waiting_to_start = True
        
        # Smoothing parameters
        speed_history = [0.0] * 3  # Reduced buffer size for quicker response
        deadband = 0.03  # Reduced deadband for more responsiveness
        min_pwm_change = 0.003  # Reduced threshold for more frequent updates
        
        # Debug counters
        deadband_count = 0
        update_count = 0
        
        # Main control loop
        last_time = time.time()
        
        while True:
            # Get current time
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            
            # Get GPS data and raw speed
            gps_data = get_gps_data()
            raw_speed = get_current_speed(gps_data)
            
            # Update speed history (for moving average filter)
            speed_history.append(raw_speed)
            speed_history = speed_history[-3:]  # Keep only last 3 readings (reduced from 5)
            
            # Calculate smoothed speed (moving average)
            current_speed = sum(speed_history) / len(speed_history)
            
            # Ensure a minimum PWM when first starting (to overcome inertia)
            time_since_last_update = current_time - last_pwm_update_time
            
            # Display status
            stdscr.clear()
            stdscr.addstr(0, 0, "GPS Speed Controller (Enhanced)")
            stdscr.addstr(1, 0, f"Current Speed: {format_value(current_speed)} m/s (raw: {format_value(raw_speed)})")
            stdscr.addstr(2, 0, f"Target Speed: {format_value(target_speed)} m/s")
            stdscr.addstr(3, 0, f"PWM Value: {format_value(current_pwm)} (dir: {direction})")
            
            if gps_data:
                # Display heading
                heading = gps_data.get('dual_gps', {}).get('bearing', 'N/A')
                stdscr.addstr(5, 0, f"Heading: {format_value(heading)}°")
                
                # Display control status
                if running:
                    error = target_speed - current_speed
                    p_term = kp * error
                    i_term = ki * integral
                    d_term = kd * (error-last_error)/dt if dt > 0 else 0
                    
                    stdscr.addstr(4, 0, f"Error: {format_value(error)} m/s")
                    stdscr.addstr(6, 0, f"P: {format_value(p_term)} I: {format_value(i_term)} D: {format_value(d_term)}")
                    
                    # Show deadband and update status
                    in_deadband = abs(error) < deadband
                    stdscr.addstr(7, 0, f"Deadband: {'Active' if in_deadband else 'Inactive'} ({deadband_count}/{update_count})")
                    stdscr.addstr(10, 0, f"Time since PWM update: {time_since_last_update:.1f}s")
            
            # Display controls
            status_line = "SPACE: Start/Stop | Q: Quit | +/-: Adjust Target Speed"
            if waiting_to_start:
                status_line = "Press SPACE to start control | Q: Quit"
            if running:
                status_line = "Running - SPACE: Stop | Q: Quit | +/-: Adjust Target Speed"
            
            stdscr.addstr(height-2, 0, status_line)
            stdscr.refresh()
            
            # Check for key presses
            key = stdscr.getch()
            if key == ord(' '):
                if waiting_to_start:
                    waiting_to_start = False
                    running = True
                    logger.info("Controller started")
                    stdscr.addstr(height-3, 0, "Controller started")
                    # Reset PID values when starting
                    integral = 0.0
                    last_error = 0.0
                    speed_history = [0.0] * 3
                    deadband_count = 0
                    update_count = 0
                    # Apply a small initial PWM to overcome inertia
                    if target_speed > 0:
                        current_pwm = 0.1
                        motor.set_speed(direction * current_pwm)
                        last_pwm_update_time = current_time
                        logger.info(f"Applied initial PWM: {current_pwm}")
                elif running:
                    running = False
                    # Stop the motor
                    motor.set_speed(0.0)
                    current_pwm = 0.0
                    logger.info("Controller stopped")
                    stdscr.addstr(height-3, 0, "Controller stopped")
                else:
                    running = True
                    logger.info("Controller resumed")
                    stdscr.addstr(height-3, 0, "Controller resumed")
            elif key == ord('q') or key == ord('Q'):
                logger.info("Controller quit by user")
                motor.set_speed(0.0)
                break
            elif key == ord('+') or key == ord('='):
                target_speed += 0.1
                logger.info(f"Target speed increased to {target_speed}")
            elif key == ord('-') or key == ord('_'):
                target_speed = max(0.1, target_speed - 0.1)
                logger.info(f"Target speed decreased to {target_speed}")
            
            # Only update motor control if running
            if running:
                update_count += 1
                
                # Calculate error
                error = target_speed - current_speed
                
                # Force minimum PWM if speed is near zero and target is not
                force_update = False
                if current_speed < 0.05 and target_speed > 0.1 and current_pwm < 0.1:
                    stdscr.addstr(8, 0, "Applying minimum PWM to overcome inertia")
                    new_pwm = 0.15  # Minimum PWM to get moving
                    force_update = True
                # Apply deadband - ignore very small errors
                elif abs(error) < deadband:
                    # Within deadband - use small control action or none
                    deadband_count += 1
                    control = 0.0
                    # Don't accumulate integral in deadband
                    stdscr.addstr(8, 0, f"In deadband - holding steady ({deadband_count} cycles)")
                    
                    # If we've been in deadband too long, force a small adjustment
                    if time_since_last_update > 5.0 and abs(error) > 0.01:
                        stdscr.addstr(9, 0, "Forcing update after deadband timeout")
                        control = 0.01 if error > 0 else -0.01  # Tiny nudge in right direction
                        force_update = True
                else:
                    # Update integral term (with anti-windup)
                    integral += error * dt
                    integral = max(-0.5, min(0.5, integral))  # More conservative integral clamp
                    
                    # Calculate derivative term with smoothing
                    if dt > 0:
                        derivative = (error - last_error) / dt
                        # Low-pass filter the derivative term
                        derivative = 0.3 * derivative  # Further dampen derivative response
                    else:
                        derivative = 0.0
                    
                    last_error = error
                    
                    # Calculate PID output
                    control = kp * error + ki * integral + kd * derivative
                
                # Very gradual change for smoother response
                max_change_per_update = 0.04  # Increased for more responsiveness
                control_change = max(-max_change_per_update, min(max_change_per_update, control))
                
                # Update PWM value
                new_pwm = current_pwm + control_change if not force_update else new_pwm
                new_pwm = max(-0.7, min(0.7, new_pwm))  # Limit PWM
                
                # Only update motor if PWM changed enough or force update is triggered
                if force_update or abs(new_pwm - current_pwm) > min_pwm_change:
                    # Apply weighted smoothing to PWM changes (more weight to new value)
                    current_pwm = 0.7 * current_pwm + 0.3 * new_pwm  # More influence from new value
                    motor_pwm = direction * current_pwm
                    motor.set_speed(motor_pwm)
                    logger.info(f"PWM set to {motor_pwm:.2f}")
                    last_pwm_update_time = current_time
                    stdscr.addstr(9, 0, f"PWM updated: {current_pwm:.3f}")
                else:
                    stdscr.addstr(9, 0, f"PWM change too small ({abs(new_pwm - current_pwm):.4f}) - holding")
            
            # Sleep for a short time
            time.sleep(0.1)  # 10Hz control loop
        
        # Clean exit message
        stdscr.clear()
        stdscr.addstr(0, 0, "Controller stopped. Press any key to exit.")
        stdscr.refresh()
        stdscr.nodelay(False)
        stdscr.getch()
        
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        logger.error(traceback.format_exc())
        if 'motor' in locals():
            motor.set_speed(0.0)
        
        try:
            stdscr.clear()
            stdscr.addstr(0, 0, f"ERROR: {str(e)}")
            stdscr.addstr(1, 0, "Press any key to exit")
            stdscr.refresh()
            stdscr.nodelay(False)
            stdscr.getch()
        except:
            pass
    
    finally:
        # Ensure motor is stopped
        if 'motor' in locals():
            motor.set_speed(0.0)
        
        # Clean up curses
        try:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.curs_set(1)  # Show cursor
        except:
            pass
        
        logger.info("Speed controller terminated")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="GPS-based speed controller")
    parser.add_argument("-r", "--reverse", action="store_true", help="Run in reverse mode")
    parser.add_argument("-s", "--speed", type=float, default=0.5, help="Target speed in m/s")
    args = parser.parse_args()

    logger.info(f"GPS Speed Controller starting (v1.0)")
    
    try:
        # Run the controller with curses wrapper
        curses.wrapper(speed_controller, reverse=args.reverse, target_speed=args.speed)
    except Exception as e:
        logger.error(f"Failed to run speed controller: {e}")
        import traceback
        logger.error(traceback.format_exc())