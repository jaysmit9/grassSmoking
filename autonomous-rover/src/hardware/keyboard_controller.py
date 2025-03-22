import threading
import time
import sys
import termios
import tty
import select
import logging

# Configure logging
logger = logging.getLogger("rover.hardware.keyboard")

class KeyboardController:
    """
    A keyboard controller implementation
    
    This controller handles keyboard input and forwards commands to the motor controller,
    it is suitable for manual joystick-like control.
    """
    
    def __init__(self, motor_controller, gps_monitor=None):
        """Initialize the keyboard controller"""
        self.motor_controller = motor_controller
        self.gps_monitor = gps_monitor
        self.running = False
        self.thread = None
        self.old_settings = None
        
        # Key mappings for control - now using arrow keys
        self.key_mappings = {
            '\x1b[A': self._forward,         # Up arrow
            '\x1b[B': self._backward,        # Down arrow
            '\x1b[D': self._steer_left,      # Left arrow
            '\x1b[C': self._steer_right,     # Right arrow
            'q': self._increase_steering_strength,
            'e': self._decrease_steering_strength,
            'c': self._center_steering,
            'x': self._stop,
            ' ': self._emergency_stop
        }
        
    def start(self):
        """Start the keyboard controller in a separate thread"""
        if self.thread is not None and self.thread.is_alive():
            logger.warning("Keyboard controller already running")
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._keyboard_loop)
        self.thread.daemon = True  # Allow program to exit even if thread is running
        self.thread.start()
        
        logger.info("Keyboard controller started. Press h for help.")
        self._print_help()
        
    def stop(self):
        """Stop the keyboard controller"""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        
        # Restore terminal settings
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            
        logger.info("Keyboard controller stopped")
        
    def _keyboard_loop(self):
        """Main loop for keyboard input"""
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                # Check if a key is available
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    # Handle quit
                    if key == 'h':
                        self._print_help()
                    elif key == '\x03':  # Ctrl+C
                        logger.info("Keyboard controller received Ctrl+C")
                        self.running = False
                    elif key == '\x1b':  # Escape sequence for arrow keys
                        # Read the next two characters for arrow keys
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            key += sys.stdin.read(1)
                            if key == '\x1b[' and select.select([sys.stdin], [], [], 0.1)[0]:
                                key += sys.stdin.read(1)
                                arrow_key = key
                                if arrow_key in self.key_mappings:
                                    self.key_mappings[arrow_key]()
                    elif key in self.key_mappings:
                        self.key_mappings[key]()
                        
                # Small sleep to reduce CPU usage
                time.sleep(0.05)
        finally:
            # Restore terminal settings
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def _print_help(self):
        """Print help information"""
        # Create a temporary string in normal terminal mode
        self.old_settings_backup = self.old_settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings_backup)
        
        print("\n=== KEYBOARD CONTROLLER HELP ===")
        print("↑ (Up arrow):    Increase forward speed")
        print("↓ (Down arrow):  Decrease speed / reverse")
        print("← (Left arrow):  Left steering (first press centers, second press steers left)")
        print("→ (Right arrow): Right steering")
        print("q:               Increase steering strength")
        print("e:               Decrease steering strength")
        print("c:               Center steering")
        print("x:               Stop motors")
        print("Space:           Emergency stop")
        print("h:               Show this help")
        print("Ctrl+C:          Exit")
        print("================================\n")
        
        # Restore raw mode
        tty.setraw(sys.stdin.fileno())
    
    # Control methods
    def _forward(self):
        """Increase forward speed"""
        self.motor_controller.increase_speed()
    
    def _backward(self):
        """Decrease speed / reverse"""
        self.motor_controller.decrease_speed()
    
    def _steer_left(self):
        """Steer left (with new control pattern)"""
        self.motor_controller.steer_left()
    
    def _steer_right(self):
        """Steer right (with new control pattern)"""
        self.motor_controller.steer_right()
    
    def _increase_steering_strength(self):
        """Increase steering strength/sensitivity"""
        self.motor_controller.increase_steering_strength()
    
    def _decrease_steering_strength(self):
        """Decrease steering strength/sensitivity"""
        self.motor_controller.decrease_steering_strength()
    
    def _center_steering(self):
        """Center the steering"""
        self.motor_controller.center_steering()
    
    def _stop(self):
        """Stop the motors"""
        self.motor_controller.stop()
    
    def _emergency_stop(self):
        """Emergency stop - stop motors and reset steering"""
        self.motor_controller.stop()
        self.motor_controller.center_steering()
        logger.warning("EMERGENCY STOP ACTIVATED")
