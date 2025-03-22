from adafruit_servokit import ServoKit
import numpy as np
import logging
import os  # For the os.path.basename function

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global instance for singleton pattern
_motor_controller_instance = None

class MotorController:
    def __init__(self, max_speed=0.3, turn_factor=0.7):
        """Initialize the motor controller with configuration parameters.
        
        Args:
            max_speed: Maximum speed value (0.0 to 1.0)
            turn_factor: How much to reduce inside wheel speed when turning (0.0 to 1.0)
        """
        self.max_speed = max_speed
        self.turn_factor = turn_factor  # How much to slow the inside wheel during turns
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.current_steering = 0.0
        self.heading_error = 0.0
        self.servo_kit = None
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

def get_motor_controller(max_speed=0.3, turn_factor=0.7):  # Change from 0.2 to 0.3
    """Get a singleton instance of the motor controller."""
    global _motor_controller_instance
    if _motor_controller_instance is None:
        _motor_controller_instance = MotorController(max_speed=max_speed, turn_factor=turn_factor)
    return _motor_controller_instance

# Legacy compatibility functions
def set_speed(speed):
    return get_motor_controller().set_speed(speed)

def set_steering(direction_degrees):
    return get_motor_controller().set_steering(direction_degrees)

def stop():
    return get_motor_controller().stop()