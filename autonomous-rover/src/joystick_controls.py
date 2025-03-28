#!/usr/bin/env python3

import time
import signal
import sys
import logging
import os
import pygame

# Add the current directory to the path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from hardware.motor_controller import get_motor_controller
from input.keyboard_handler import get_keyboard_handler

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class JoystickHandler:
    def __init__(self):
        self.running = False
        self.joystick = None
        self.motor = get_motor_controller()
        self.use_keyboard_fallback = False
        self.keyboard_handler = None
        
        # Control parameters
        self.speed_factor = 1.0  # Maximum speed multiplier
        self.current_speed = 0.0
        self.max_speed = self.motor.max_speed
        self.steering_deadzone = 0.05  # Ignore joystick values below this threshold
        
        # Initialize pygame for joystick support
        try:
            pygame.init()
            pygame.joystick.init()
            logger.info("Pygame initialized successfully")
        except Exception as e:
            logger.error(f"Failed to initialize pygame: {e}")
        
        # Print available input devices for diagnostics
        self._print_available_devices()
        
        # Try to initialize joystick, fall back to keyboard if needed
        if not self.initialize_joystick():
            logger.warning("No joysticks found, will use keyboard fallback")
            self.use_keyboard_fallback = True
            self.keyboard_handler = get_keyboard_handler()
    
    def _print_available_devices(self):
        """Print information about available input devices for diagnostics."""
        try:
            joystick_count = pygame.joystick.get_count()
            logger.info(f"Available joysticks: {joystick_count}")
            
            if joystick_count > 0:
                for i in range(joystick_count):
                    try:
                        joystick = pygame.joystick.Joystick(i)
                        joystick.init()
                        logger.info(f"Joystick {i}: {joystick.get_name()}")
                        # Print additional joystick info
                        try:
                            logger.info(f"  - Number of axes: {joystick.get_numaxes()}")
                            logger.info(f"  - Number of buttons: {joystick.get_numbuttons()}")
                            logger.info(f"  - Number of hats: {joystick.get_numhats()}")
                        except:
                            pass
                    except Exception as e:
                        logger.error(f"Error initializing joystick {i}: {e}")
            
            # If we're on Linux, try to check input devices directly
            if os.path.exists('/dev/input'):
                logger.info("Checking input devices in /dev/input:")
                input_devices = [f for f in os.listdir('/dev/input') if f.startswith('js') or f.startswith('event')]
                if input_devices:
                    for device in input_devices:
                        logger.info(f"  - Found input device: {device}")
                else:
                    logger.info("  - No js* or event* devices found in /dev/input")
        except Exception as e:
            logger.error(f"Error listing input devices: {e}")
    
    def initialize_joystick(self):
        """Initialize the first available joystick."""
        try:
            joystick_count = pygame.joystick.get_count()
            
            if joystick_count == 0:
                logger.error("No joysticks found")
                return False
            
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logger.info(f"Initialized joystick: {self.joystick.get_name()}")
            return True
        except pygame.error as e:
            logger.error(f"Failed to initialize joystick: {e}")
            return False
        except Exception as e:
            logger.error(f"Unexpected error initializing joystick: {e}")
            return False
    
    def start(self):
        """Start the control handler."""
        if self.running:
            logger.info("Control handler already running")
            return True
        
        self.running = True
        if self.use_keyboard_fallback:
            logger.info("Starting keyboard fallback control")
            self.keyboard_handler.start()
        elif not self.joystick:
            logger.error("Cannot start joystick handler - no joystick initialized")
            self.running = False
            return False
        
        logger.info("Joystick handler started")
        return True
    
    def stop(self):
        """Stop the control handler."""
        if not self.running:
            logger.info("Control handler is already stopped")0
            return
        
        self.running = False
        self.motor.stop()
        
        if self.use_keyboard_fallback and self.keyboard_handler:
            logger.info("Stopping keyboard fallback control")
            self.keyboard_handler.stop()
        else:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except Exception as e:
                logger.error(f"Error during pygame cleanup: {e}")
                
        logger.info("Control handler stopped")
    
    def process_events(self):
        """Process control events and update motor controls."""
        if not self.running:
            return
        
        if self.use_keyboard_fallback:
            # When using keyboard fallback, we don't need to do anything here
            # as the keyboard handler runs in its own thread
            time.sleep(0.1)
            return True
        
        # Process joystick events
        try:
            pygame.event.pump()
            
            # Get joystick axis values
            y_axis = self.joystick.get_axis(1)
            x_axis = self.joystick.get_axis(0)
            
            # Speed control (Y-axis inverted as pushing forward is negative)
            target_speed = -y_axis * self.max_speed * self.speed_factor
            
            # Apply speed changes gradually for smoother control
            if abs(target_speed - self.current_speed) > 0.01:
                if target_speed > self.current_speed:
                    self.current_speed = min(self.current_speed + 0.05, target_speed)
                else:
                    self.current_speed = max(self.current_speed - 0.05, target_speed)
                
                self.motor.set_speed(self.current_speed)
                logger.debug(f"Speed: {self.current_speed:.2f}")
            
            # Steering control (X-axis)
            if abs(x_axis) > self.steering_deadzone:
                if abs(x_axis) > 0.75:
                    steering_position = 2 if x_axis > 0 else -2
                elif abs(x_axis) > 0.25:
                    steering_position = 1 if x_axis > 0 else -1
                else:
                    steering_position = 0
                
                if steering_position != self.motor.steering_position:
                    if steering_position > self.motor.steering_position:
                        while self.motor.steering_position < steering_position:
                            self.motor.steer_right()
                    else:
                        while self.motor.steering_position > steering_position:
                            self.motor.steer_left()
                    
                    logger.debug(f"Steering position: {self.motor.steering_position}")
            elif abs(x_axis) <= self.steering_deadzone and self.motor.steering_position != 0:
                self.motor.center_steering()
                logger.debug("Steering centered")
            
            # Check for stop button (usually button 0)
            if self.joystick.get_button(0):
                logger.info("Emergency stop button pressed")
                self.motor.stop()
                self.current_speed = 0.0
            
            return True
            
        except Exception as e:
            logger.error(f"Error processing joystick events: {e}")
            return False

def signal_handler(sig, frame):
    """Handle Ctrl+C to exit gracefully."""
    logger.info("Stopping the rover...")
    control_handler.stop()
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize motor controller
    motor = get_motor_controller()
    
    # Initialize control handler (either joystick or keyboard)
    control_handler = JoystickHandler()
    
    if control_handler.start():
        control_type = "keyboard fallback" if control_handler.use_keyboard_fallback else "joystick"
        logger.info(f"Rover ready for {control_type} control")
        logger.info("Press Ctrl+C to exit")
        
        # Main loop
        try:
            while control_handler.running:
                control_handler.process_events()
                time.sleep(0.05)  # 20Hz update rate
        except KeyboardInterrupt:
            signal_handler(None, None)
    else:
        logger.error("Failed to start control handler")
        sys.exit(1)
