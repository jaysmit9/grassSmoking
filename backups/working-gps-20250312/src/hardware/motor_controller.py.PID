from adafruit_servokit import ServoKit
import time
import logging
import os  # Add this for the os.path.basename function

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global instance for singleton pattern
_motor_controller_instance = None

class MotorController:
    def __init__(self, max_speed=0.2, turn_factor=0.9):
        """Initialize the motor controller with configuration parameters."""
        self.max_speed = max_speed
        self.turn_factor = turn_factor
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
            logger.error(f"Failed to initialize motors: {e}")
            return False
    
    def set_speed(self, speed):
        """Set the speed of the rover's motors while preserving steering."""
        # Debug where this is being called from
        import traceback
        caller = traceback.extract_stack()[-2]
        logger.info(f"set_speed({speed:.2f}) called from {os.path.basename(caller.filename)}:{caller.lineno}")
        
        # Ensure speed is within bounds
        speed = max(-self.max_speed, min(self.max_speed, speed))
        
        # Check if we have active steering
        if hasattr(self, 'current_steering') and abs(self.current_steering) > 0.1:
            logger.info(f"Preserving steering differential for angle {self.current_steering:.1f}°")
            
            # Calculate steering factor again
            steering_factor = max(-1, min(1, self.current_steering / 35.0))
            
            # Apply differential steering based on the new target speed
            if steering_factor < 0:  # Turn left
                self.left_speed = speed * (1 - self.turn_factor * abs(steering_factor))
                self.right_speed = speed
            elif steering_factor > 0:  # Turn right
                self.left_speed = speed
                self.right_speed = speed * (1 - self.turn_factor * abs(steering_factor))
            else:
                self.left_speed = speed
                self.right_speed = speed
                
            logger.info(f"After speed adjust w/steering: L={self.left_speed:.2f}, R={self.right_speed:.2f}")
        else:
            # No steering, set both motors equal
            self.left_speed = speed
            self.right_speed = speed
        
        # Apply the speed to motors
        self._apply_motor_speeds()
        
        return True
    
    def set_steering(self, direction_degrees):
        """
        Set the steering direction in degrees.
        Negative = LEFT turn, Positive = RIGHT turn, 0 = straight
        """
        # Store current steering angle
        self.current_steering = direction_degrees
        
        # Get current base speed
        base_speed = self.max_speed
        
        # SIMPLIFIED APPROACH: Use direct logic with clear comments
        if direction_degrees < 0:  # NEGATIVE = LEFT TURN
            # For LEFT turn: LEFT motor slower, RIGHT motor faster
            left_speed = max(0, base_speed * 0.1)  # Very slow left motor
            right_speed = base_speed  # Full speed right motor
            logger.info(f"LEFT TURN: {direction_degrees:.1f}°")
        elif direction_degrees > 0:  # POSITIVE = RIGHT TURN
            # For RIGHT turn: RIGHT motor slower, LEFT motor faster
            left_speed = base_speed  # Full speed left motor
            right_speed = max(0, base_speed * 0.1)  # Very slow right motor
            logger.info(f"RIGHT TURN: {direction_degrees:.1f}°")
        else:  # Go straight
            left_speed = base_speed
            right_speed = base_speed
            logger.info("STRAIGHT")
        
        # Store the calculated speeds
        self.left_speed = left_speed
        self.right_speed = right_speed
        
        # Apply the speeds to motors
        self._apply_motor_speeds()
        
        return True
    
    def _apply_motor_speeds(self):
        """Apply the calculated speeds to the actual motors."""
        try:
            if self.servo_kit:
                # Debug the values we're trying to send
                logger.info(f"Applying to motors - Left: {self.left_speed:.2f}, Right: {self.right_speed:.2f}")
                
                # Ensure values are within appropriate ranges (-1.0 to 1.0)
                left = max(-1.0, min(1.0, self.left_speed))
                right = max(-1.0, min(1.0, self.right_speed))
                
                # Apply to servo channels
                self.servo_kit.continuous_servo[1].throttle = left
                self.servo_kit.continuous_servo[0].throttle = right
                
                # Verify the values were set correctly
                time.sleep(0.01)  # Small delay to allow values to be set
                logger.info(f"Motor values set - Left: {self.servo_kit.continuous_servo[1].throttle}, "
                           f"Right: {self.servo_kit.continuous_servo[0].throttle}")
                
                return True
            else:
                logger.error("Servo kit not initialized")
                return False
        except Exception as e:
            logger.error(f"Error setting motor speeds: {e}")
            return False
    
    def stop(self):
        """Stop the rover's motors."""
        try:
            self.left_speed = 0
            self.right_speed = 0
            if self.servo_kit:
                self.servo_kit.continuous_servo[0].throttle = 0
                self.servo_kit.continuous_servo[1].throttle = 0
            logger.info("Motors stopped")
            return True
        except Exception as e:
            logger.error(f"Error stopping motors: {e}")
            return False

    def get_status(self):
        """Get current motor status."""
        return {
            'left_speed': self.left_speed,
            'right_speed': self.right_speed,
            'steering': self.current_steering,
            'heading_error': self.heading_error
        }

    def set_heading_error(self, error_degrees):
        """Store the current heading error for logging purposes."""
        self.heading_error = error_degrees

# Singleton getter function
def get_motor_controller():
    """Get or create the singleton motor controller instance."""
    global _motor_controller_instance
    if _motor_controller_instance is None:
        _motor_controller_instance = MotorController()
    return _motor_controller_instance

# Backward compatibility functions that use the singleton instance
def set_motor_speed(speed):
    """Legacy function for setting motor speed."""
    controller = get_motor_controller()
    return controller.set_speed(speed)

def set_motor_direction(direction):
    """Legacy function for setting motor direction."""
    controller = get_motor_controller()
    return controller.set_steering(direction)

def stop_motors():
    """Legacy function for stopping motors."""
    controller = get_motor_controller()
    return controller.stop()

def initialize_motors():
    """Legacy function for initializing motors."""
    controller = get_motor_controller()
    return controller.initialize()

def update_motor_controls(speed, direction):
    """Legacy function for updating motor controls."""
    controller = get_motor_controller()
    # IMPORTANT: Set steering first, THEN speed
    # This way speed won't override the steering settings
    controller.set_steering(direction)
    # Don't set speed again, as it would override steering
    return True