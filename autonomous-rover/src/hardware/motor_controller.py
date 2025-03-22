from adafruit_servokit import ServoKit
import numpy as np
import logging
import os  # For the os.path.basename function
import time
import threading

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global instance for singleton pattern
_motor_controller_instance = None

class MotorController:
<<<<<<< HEAD
    def __init__(self, max_speed=0.2, turn_factor=0.9, simulation_mode=False):
        """Initialize the motor controller with configuration parameters."""
        self.max_speed = max_speed
        self.turn_factor = turn_factor
        self.simulation_mode = simulation_mode
=======
    def __init__(self, max_speed=0.3, turn_factor=0.7):
        """Initialize the motor controller with configuration parameters.
        
        Args:
            max_speed: Maximum speed value (0.0 to 1.0)
            turn_factor: How much to reduce inside wheel speed when turning (0.0 to 1.0)
        """
        self.max_speed = max_speed
        self.turn_factor = turn_factor  # How much to slow the inside wheel during turns
>>>>>>> gpsd-refactor
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.current_steering = 0.0
        self.current_speed = 0.0  # Add this for compatibility with joystick controls
        self.heading_error = 0.0
        self.servo_kit = None
        
        # Add variables for new joystick-like steering control
        self.steering_position = 0  # -2, -1, 0, 1, 2 (left to right)
        self.steering_multiplier = 2.0
        self.max_steering_multiplier = 4.0
        self.steering_step = 0.5  # Each button press changes steering by this amount
        self._left_factor = 1.0
        self._right_factor = 1.0
        
        self.initialize()
    
    def initialize(self):
        """Initialize motor controllers."""
        try:
            self.servo_kit = ServoKit(channels=16)
            self.servo_kit.continuous_servo[0].throttle = 0  # Right servo
            self.servo_kit.continuous_servo[1].throttle = 0  # Left servo
            logger.info("Motor controller initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to initialize motor controller: {e}")
            return False
    
    def set_speed(self, speed):
        """Set the speed of the rover's motors while preserving steering.
        
        Args:
            speed: Speed value from -max_speed to max_speed
                  Negative values mean reverse.
        """
        # Bound speed to max_speed
        speed = np.clip(speed, -self.max_speed, self.max_speed)
        logger.debug(f"Setting speed: {speed}")
        
        # Apply current steering with the new speed
        self._apply_steering_and_speed(self.current_steering, speed)
        return True
    
    def set_steering(self, angle_degrees):
        """
<<<<<<< HEAD
        Set steering with smooth proportional differential response
        """
        # Store current steering
        self.current_steering = direction_degrees
        
        # IMPROVED: Use a smooth, truly proportional response curve
        # Convert degrees to a normalized differential factor
        # This gives a nice sigmoidal response curve that's gentle for small angles
        # but appropriately strong for larger angles
        
        # Parameters for response curve
        max_differential = 0.90   # Maximum differential at extreme angles
        sensitivity = 0.05        # Higher = more sensitive to small angles
        
        # Calculate normalized differential factor using sigmoid-like function
        # This creates a smooth S-curve response from 0 to max_differential
        normalized_angle = abs(direction_degrees) / 45.0  # Normalize to [0,1] range at 45 degrees
        differential_factor = max_differential * (normalized_angle / (sensitivity + normalized_angle))
        
        # Calculate motor speeds with true proportional response
        base_speed = self.max_speed
        
        if direction_degrees < 0:  # LEFT TURN
            # Left turn: reduce left motor proportionally
            left_factor = 1.0 - differential_factor
            right_factor = 1.0
            logger.info(f"LEFT TURN: {direction_degrees:.1f}° (differential: {differential_factor:.3f})")
        elif direction_degrees > 0:  # RIGHT TURN
            # Right turn: reduce right motor proportionally
            left_factor = 1.0
            right_factor = 1.0 - differential_factor
            logger.info(f"RIGHT TURN: {direction_degrees:.1f}° (differential: {differential_factor:.3f})")
        else:  # STRAIGHT
            # Perfectly straight
            left_factor = 1.0
            right_factor = 1.0
        
        # Calculate final motor speeds
        left_speed = base_speed * left_factor
        right_speed = base_speed * right_factor
        
        # Add diagnostic output to understand response curve
        logger.info(f"Motor speeds: L={left_speed:.3f}, R={right_speed:.3f}, Ratio: {left_speed/right_speed if right_speed > 0 else 'inf':.3f}")
        
        # Store and apply motor speeds
        self.left_speed = left_speed
        self.right_speed = right_speed
        self._apply_motor_speeds()
=======
        Set steering angle in degrees (-90 to +90)
        Positive angle turns right, negative angle turns left
        """
        # Clamp angle to valid range
        angle_degrees = max(-90, min(90, angle_degrees))
        
        # Debug the input
        logger.warning(f"MOTOR DEBUG - Raw steering command: {angle_degrees:.1f}°")
        
        # Determine turn direction for logging
        if abs(angle_degrees) < 2:
            direction = "STRAIGHT"
        elif angle_degrees > 0:
            direction = "RIGHT TURN"
        else:
            direction = "LEFT TURN"
        
        logger.info(f"{direction}: {abs(angle_degrees):.1f}°")
        
        # Calculate differential steering - add detailed logging
        if angle_degrees > 0:  # Turn right
            left_factor = 1.0  # Left wheel at full speed
            right_factor = 1.0 - 2 * (angle_degrees / 180.0)  # Right wheel slows down
            logger.warning(f"MOTOR DEBUG - RIGHT turn factors: L={left_factor:.3f}, R={right_factor:.3f}")
        else:  # Turn left
            right_factor = 1.0  # Right wheel at full speed
            left_factor = 1.0 + 2 * (angle_degrees / 180.0)  # Left wheel slows down
            logger.warning(f"MOTOR DEBUG - LEFT turn factors: L={left_factor:.3f}, R={right_factor:.3f}")
        
        # Store for later use with set_speed
        self._left_factor = max(0, min(1, left_factor))
        self._right_factor = max(0, min(1, right_factor))
    
    def _apply_steering_and_speed(self, steering_degrees, speed):
        """Apply steering and speed settings to motors.
        
        Args:
            steering_degrees: Steering angle in degrees
            speed: Target speed value (positive or negative)
        """
        # Store the current steering angle and speed
        self.current_steering = steering_degrees
        
        # Bound speed to max_speed
        speed = np.clip(speed, -self.max_speed, self.max_speed)
        
        # FIXED: Use the precalculated factors from set_steering
        # These factors already account for steering - no need to recalculate
        left_speed = speed * self._left_factor
        right_speed = speed * self._right_factor
        
        # Debug the actual application
        logger.warning(f"MOTOR DEBUG - Speed={speed:.2f}, Factors: L={self._left_factor:.2f}, R={self._right_factor:.2f}")
        logger.warning(f"MOTOR DEBUG - Final speeds: L={left_speed:.2f}, R={right_speed:.2f}")
        
        # Update the stored speed values
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        logger.info(f"Applying to motors - Left: {self.left_speed:.2f}, Right: {self.right_speed:.2f}")
>>>>>>> gpsd-refactor
        
        # Set the motors
        try:
            self.servo_kit.continuous_servo[1].throttle = self.left_speed
            self.servo_kit.continuous_servo[0].throttle = self.right_speed
            logger.debug(f"Motor values set - Left: {self.left_speed}, Right: {self.right_speed}")
            return True
        except Exception as e:
            logger.error(f"Error setting motor speeds: {e}")
            return False
    
    def stop(self):
        """Stop all motors."""
        try:
            self.servo_kit.continuous_servo[0].throttle = 0
            self.servo_kit.continuous_servo[1].throttle = 0
            self.left_speed = 0.0
            self.right_speed = 0.0
            logger.info("Motors stopped")
            return True
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
            return False
    
    def set_heading_error(self, error_degrees):
        """Store current heading error for logging/debugging."""
        self.heading_error = error_degrees
    
    def get_status(self):
        """Get current motor controller status."""
        return {
            'left_speed': self.left_speed,
            'right_speed': self.right_speed,
            'current_steering': self.current_steering,
            'heading_error': self.heading_error
        }
    
    # Add new joystick-like control methods
    def increase_speed(self):
        """Increase the speed by a fixed increment"""
        if self.current_speed < 0:
            # If going backward, first go to zero
            speed = 0
        else:
            # Otherwise increase speed
            speed = min(self.current_speed + 0.05, self.max_speed)
        
        self.current_speed = speed
        logger.info(f"Speed increased to {self.current_speed:.2f}")
        return self.set_speed(speed)
    
    def decrease_speed(self):
        """Decrease the speed by a fixed increment"""
        if self.current_speed > 0:
            # If going forward, first go to zero
            speed = 0
        else:
            # Otherwise increase reverse speed
            speed = max(self.current_speed - 0.05, -self.max_speed)
        
        self.current_speed = speed
        logger.info(f"Speed decreased to {self.current_speed:.2f}")
        return self.set_speed(speed)
    
    # Add joystick-like steering methods
    def steer_left(self):
        """New steering left method that turns left incrementally"""
        if self.steering_position > -2:
            self.steering_position -= 1
            self._set_steering_from_position()
            logger.info(f"Steering position: {self.steering_position} ({self.current_steering:.1f}°)")
        return self.current_steering
    
    def steer_right(self):
        """New steering right method that turns right incrementally"""
        if self.steering_position < 2:
            self.steering_position += 1
            self._set_steering_from_position()
            logger.info(f"Steering position: {self.steering_position} ({self.current_steering:.1f}°)")
        return self.current_steering
    
    def _set_steering_from_position(self):
        """Convert steering position to actual steering angle"""
        # Position to angle: -2 -> -30°, -1 -> -15°, 0 -> 0°, 1 -> 15°, 2 -> 30°
        base_angle = self.steering_position * 15.0
        
        # Apply steering multiplier for strength control
        angle = base_angle * self.steering_multiplier
        self.set_steering(angle)
        return angle
    
    def increase_steering_strength(self):
        """Increase the steering strength multiplier"""
        self.steering_multiplier = min(self.steering_multiplier + self.steering_step, self.max_steering_multiplier)
        if self.steering_position != 0:
            self._set_steering_from_position()  # Recalculate steering if we're not centered
        logger.info(f"Steering strength increased to {self.steering_multiplier:.1f}x")
        return self.steering_multiplier
    
    def decrease_steering_strength(self):
        """Decrease the steering strength multiplier"""
        self.steering_multiplier = max(self.steering_multiplier - self.steering_step, 0.5)
        if self.steering_position != 0:
            self._set_steering_from_position()  # Recalculate steering if we're not centered
        logger.info(f"Steering strength decreased to {self.steering_multiplier:.1f}x")
        return self.steering_multiplier
    
    def center_steering(self):
        """Center the steering explicitly"""
        self.steering_position = 0
        self.current_steering = 0.0
        logger.info("Steering centered")
        self.set_steering(0.0)
        return 0.0
    
    # Arrow key handler methods
    def handle_arrow_up(self):
        """Handle up arrow key press - increase forward speed"""
        logger.info("Arrow UP pressed - increasing speed")
        return self.increase_speed()
    
    def handle_arrow_down(self):
        """Handle down arrow key press - decrease speed or reverse"""
        logger.info("Arrow DOWN pressed - decreasing speed")
        return self.decrease_speed()
    
    def handle_arrow_left(self):
        """Handle left arrow key press - steer left"""
        logger.info("Arrow LEFT pressed - steering left")
        return self.steer_left()
    
    def handle_arrow_right(self):
        """Handle right arrow key press - steer right"""
        logger.info("Arrow RIGHT pressed - steering right")
        return self.steer_right()

def get_motor_controller(max_speed=0.3, turn_factor=2):  # Change from 0.2 to 0.3
    """Get a singleton instance of the motor controller."""
    global _motor_controller_instance
<<<<<<< HEAD
    if (_motor_controller_instance is None):
        _motor_controller_instance = MotorController()
=======
    if _motor_controller_instance is None:
        _motor_controller_instance = MotorController(max_speed=max_speed, turn_factor=turn_factor)
>>>>>>> gpsd-refactor
    return _motor_controller_instance

# Legacy compatibility functions
def set_speed(speed):
    return get_motor_controller().set_speed(speed)

def set_steering(direction_degrees):
    return get_motor_controller().set_steering(direction_degrees)

def stop():
    return get_motor_controller().stop()

# Arrow key convenience functions
def handle_arrow_up():
    return get_motor_controller().handle_arrow_up()

def handle_arrow_down():
    return get_motor_controller().handle_arrow_down()

def handle_arrow_left():
    return get_motor_controller().handle_arrow_left()

def handle_arrow_right():
    return get_motor_controller().handle_arrow_right()

# ... existing code for the simulated and GPIO motor controllers ...