#!/usr/bin/env python3
import pigpio
import time
from adafruit_servokit import ServoKit

# Initialize the PCA9685
kit = ServoKit(channels=16)

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# RC input pins (avoid GPIO 15 which is used for GPS)
THROTTLE_PIN = 17  # GPIO 17
STEERING_PIN = 27  # GPIO 27

# Variables to store RC pulse widths
throttle_pulse = 1500  # Default center value (microseconds)
steering_pulse = 1500  # Default center value (microseconds)

# Variables to track pulse timing
throttle_rising_tick = 0
steering_rising_tick = 0

# RC PWM callback functions
def throttle_callback(gpio, level, tick):
    global throttle_pulse, throttle_rising_tick
    if level == 1:  # Rising edge
        throttle_rising_tick = tick
    elif level == 0:  # Falling edge
        if throttle_rising_tick != 0:
            # Calculate pulse width
            pulse_width = pigpio.tickDiff(throttle_rising_tick, tick)
            if 700 < pulse_width < 2300:  # Valid RC signal range
                throttle_pulse = pulse_width

def steering_callback(gpio, level, tick):
    global steering_pulse, steering_rising_tick
    if level == 1:  # Rising edge
        steering_rising_tick = tick
    elif level == 0:  # Falling edge
        if steering_rising_tick != 0:
            # Calculate pulse width
            pulse_width = pigpio.tickDiff(steering_rising_tick, tick)
            if 700 < pulse_width < 2300:  # Valid RC signal range
                steering_pulse = pulse_width

# Set up the callbacks
pi.set_mode(THROTTLE_PIN, pigpio.INPUT)
pi.set_mode(STEERING_PIN, pigpio.INPUT)
throttle_cb = pi.callback(THROTTLE_PIN, pigpio.EITHER_EDGE, throttle_callback)
steering_cb = pi.callback(STEERING_PIN, pigpio.EITHER_EDGE, steering_callback)

try:
    print("RC Input reading started. Press CTRL+C to exit.")
    
    # Define which PCA9685 channels to use for throttle and steering
    THROTTLE_CHANNEL = 0  # PCA9685 channel for throttle motor
    STEERING_CHANNEL = 1  # PCA9685 channel for steering servo
    
    while True:
        # Map RC pulse values (typically 1000-2000 µs) to PCA9685 values (0-180 for ServoKit)
        # For throttle (assuming bidirectional ESC or motor controller)
        throttle_value = (throttle_pulse - 1500) / 500  # Scale to -1.0 to 1.0
        throttle_value = min(max(throttle_value, -1.0), 1.0)  # Constrain
        
        # For steering (assuming standard servo)
        steering_angle = (steering_pulse - 1000) * 180 / 1000  # Scale to 0-180
        steering_angle = min(max(steering_angle, 0), 180)  # Constrain
        
        # Set the PCA9685 outputs
        if hasattr(kit, 'continuous_servo'):
            # For continuous rotation servos or ESCs
            kit.continuous_servo[THROTTLE_CHANNEL].throttle = throttle_value
        else:
            # For standard PWM (might need adjustment based on your motor controller)
            throttle_pwm = int((throttle_value + 1) * 90)  # Scale -1,1 to 0,180
            kit.servo[THROTTLE_CHANNEL].angle = throttle_pwm
        
        # Set steering servo
        kit.servo[STEERING_CHANNEL].angle = steering_angle
        
        # Print values for debugging
        print(f"Throttle: {throttle_pulse} µs ({throttle_value:.2f}), "
              f"Steering: {steering_pulse} µs ({steering_angle:.1f}°)")
        
        time.sleep(0.05)  # Small delay to prevent CPU hogging

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    # Clean up
    throttle_cb.cancel()
    steering_cb.cancel()
    pi.stop()