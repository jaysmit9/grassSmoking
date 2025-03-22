import numpy as np
import time

class SimulatedMotorController:
    def __init__(self, max_speed=2.0):
        self.max_speed = max_speed
        self.current_speed = 0.0
        self.current_steering = 0.0
        print("Simulated Motor Controller initialized")
    
    def set_speed(self, speed):
        """Set the speed of the simulated motors."""
        # Limit speed to max_speed
        speed = np.clip(speed, -self.max_speed, self.max_speed)
        self.current_speed = speed
        return True
    
    def set_steering(self, direction_degrees):
        """Set the steering direction in degrees."""
        # Store current steering angle
        self.current_steering = direction_degrees
        
        # Get base speed
        base_speed = self.current_speed if self.current_speed != 0 else self.max_speed
        
        # Convert degrees to a steering factor (-1 to 1)
        # -1 = full left, 0 = straight, 1 = full right
        steering_factor = max(-1, min(1, direction_degrees / 50.0))
        
        # Apply differential steering
        if steering_factor < 0:  # Turn left
            # Reduce left motor speed based on steering angle
            self.left_speed = base_speed * (1 - 0.7 * abs(steering_factor))
            self.right_speed = base_speed
        elif steering_factor > 0:  # Turn right
            self.left_speed = base_speed
            # Reduce right motor speed based on steering angle
            self.right_speed = base_speed * (1 - 0.7 * abs(steering_factor))
        else:  # Go straight
            self.left_speed = base_speed
            self.right_speed = base_speed
        
        print(f"SIM Steering: {direction_degrees:.1f}°, Left: {self.left_speed:.2f}, Right: {self.right_speed:.2f}")
        
        return True
    
    def stop(self):
        """Stop the motors."""
        self.current_speed = 0.0
        return True
    
    def get_status(self):
        """Get current motor status."""
        return {
            'speed': self.current_speed,
            'steering': self.current_steering
        }

def main():
    """Test the simulated motor controller."""
    motors = SimulatedMotorController()
    
    try:
        print("Testing motor controller...")
        print("Setting speed to 1.5...")
        motors.set_speed(1.5)
        print(f"Status: {motors.get_status()}")
        
        print("\nSetting steering to 30 degrees right...")
        motors.set_steering(30)
        print(f"Status: {motors.get_status()}")
        
        print("\nSetting steering to -25 degrees left...")
        motors.set_steering(-25)
        print(f"Status: {motors.get_status()}")
        
        print("\nStopping motors...")
        motors.stop()
        print(f"Status: {motors.get_status()}")
        
    except KeyboardInterrupt:
        print("\nTest interrupted.")
        motors.stop()

if __name__ == "__main__":
    main()