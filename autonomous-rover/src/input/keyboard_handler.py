import curses
import threading
import time
import logging
import sys
import os

# Add the parent directory to sys.path to allow absolute imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.motor_controller import get_motor_controller, stop

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class KeyboardHandler:
    def __init__(self):
        self.running = False
        self.thread = None
        self.stdscr = None
        self.motor = get_motor_controller()
        
        # Control parameters
        self.current_speed = 0.0
        self.speed_change = 0.05
        self.max_speed = self.motor.max_speed
        self.turning = 0  # -1: left, 0: straight, 1: right
    
    def start(self):
        """Start the keyboard handler in a separate thread."""
        if self.thread is not None and self.thread.is_alive():
            logger.info("Keyboard handler already running")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._keyboard_loop)
        self.thread.daemon = True  # Thread will exit when main program exits
        self.thread.start()
        logger.info("Keyboard handler started")
    
    def stop(self):
        """Stop the keyboard handler."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        if self.stdscr:
            try:
                curses.nocbreak()
                self.stdscr.keypad(False)
                curses.echo()
                curses.endwin()
                self.stdscr = None
            except:
                pass  # If curses cleanup fails, just continue
        
        logger.info("Keyboard handler stopped")
    
    def _keyboard_loop(self):
        """Main keyboard input loop using the approach from gpsd_gps_monitor.py."""
        try:
            # Initialize curses
            self.stdscr = curses.initscr()
            curses.noecho()
            curses.cbreak()
            self.stdscr.keypad(True)  # Enable special keys like arrow keys
            self.stdscr.nodelay(True)  # Non-blocking input
            
            logger.info("Keyboard handler running - use arrow keys to drive")
            self.stdscr.addstr(0, 0, "Use arrow keys to drive the rover. Press 'q' to quit, spacebar to stop.")
            self.stdscr.refresh()
            
            while self.running:
                try:
                    # Get key input
                    key = self.stdscr.getch()
                    
                    if key == ord('q'):  # Quit on 'q'
                        logger.info("Quit command received")
                        self.running = False
                        self.motor.stop()
                    elif key == ord(' '):  # Space bar to stop
                        logger.info("STOP command (spacebar)")
                        self.current_speed = 0.0
                        self.motor.set_speed(0)
                    # Arrow Up/Down: Change speed - directly using the code from gpsd_gps_monitor.py
                    elif key == curses.KEY_UP:
                        self.current_speed += self.speed_change
                        self.current_speed = min(self.current_speed, self.max_speed)
                        logger.info(f"UP arrow - speed: {self.current_speed:.2f}")
                        self.motor.set_speed(self.current_speed)
                    elif key == curses.KEY_DOWN:
                        self.current_speed -= self.speed_change
                        self.current_speed = max(self.current_speed, -self.max_speed)
                        logger.info(f"DOWN arrow - speed: {self.current_speed:.2f}")
                        self.motor.set_speed(self.current_speed)
                    # Arrow Left/Right: Turn - using the approach from gpsd_gps_monitor.py
                    elif key == curses.KEY_LEFT:
                        # Set turning state for display
                        self.turning = self.motor.steering_position - 1
                        logger.info(f"LEFT arrow - turning left, position: {self.turning}")
                        self.motor.steer_left()
                        # Refresh motor speed after steering change
                        self.motor.set_speed(self.current_speed)
                    elif key == curses.KEY_RIGHT:
                        # Set turning state for display
                        self.turning = self.motor.steering_position + 1
                        logger.info(f"RIGHT arrow - turning right, position: {self.turning}")
                        self.motor.steer_right()
                        # Refresh motor speed after steering change
                        self.motor.set_speed(self.current_speed)
                    elif key == ord('c'):  # 'c' to center steering
                        self.turning = 0
                        logger.info("Centering steering")
                        self.motor.center_steering()
                        # Refresh motor speed after steering change
                        self.motor.set_speed(self.current_speed)
                    
                    # Short sleep to prevent CPU hogging
                    time.sleep(0.05)
                    
                    # Update display
                    self._update_display()
                    
                except curses.error:
                    # No input available
                    time.sleep(0.1)
                    
        except Exception as e:
            logger.error(f"Error in keyboard handler: {e}")
        finally:
            # Clean up curses
            self.stop()
    
    def _update_display(self):
        """Update the terminal display with current status."""
        try:
            self.stdscr.clear()
            self.stdscr.addstr(0, 0, "Rover Keyboard Control")
            self.stdscr.addstr(1, 0, "===================")
            self.stdscr.addstr(2, 0, f"Speed: {self.current_speed:.2f}")
            
            # Show steering as a numerical value instead of text
            self.stdscr.addstr(3, 0, f"Steering: {self.motor.current_steering:.2f}° (pos: {self.motor.steering_position})")
            
            # Add motor PWM levels
            self.stdscr.addstr(4, 0, f"Left motor: {self.motor.left_speed:.3f}  Right motor: {self.motor.right_speed:.3f}")
            
            # Instructions
            self.stdscr.addstr(6, 0, "Controls:")
            self.stdscr.addstr(7, 0, "- UP/DOWN: Adjust speed")
            self.stdscr.addstr(8, 0, "- LEFT/RIGHT: Turn")
            self.stdscr.addstr(9, 0, "- SPACE: Stop")
            self.stdscr.addstr(10, 0, "- c: Center steering")
            self.stdscr.addstr(11, 0, "- q: Quit")
            
            self.stdscr.refresh()
        except:
            pass  # Ignore display errors

# Convenience function to get a keyboard handler instance
_keyboard_handler = None
def get_keyboard_handler():
    global _keyboard_handler
    if _keyboard_handler is None:
        _keyboard_handler = KeyboardHandler()
    return _keyboard_handler
