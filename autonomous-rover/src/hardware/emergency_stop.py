import RPi.GPIO as GPIO
import threading
import time

class EmergencyStop:
    def __init__(self, pin=17):
        """Initialize emergency stop button on specified GPIO pin"""
        self.pin = pin
        self.stop_requested = False
        self._setup_gpio()
        self._start_monitoring()
        
    def _setup_gpio(self):
        """Set up the GPIO pin for the emergency stop button"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    def _button_callback(self, channel):
        """Callback function when button is pressed"""
        self.stop_requested = True
        print("EMERGENCY STOP ACTIVATED!")
    
    def _start_monitoring(self):
        """Start monitoring the emergency stop button"""
        GPIO.add_event_detect(self.pin, GPIO.FALLING, 
                             callback=self._button_callback, 
                             bouncetime=300)
    
    def is_activated(self):
        """Check if emergency stop has been activated"""
        return self.stop_requested
    
    def reset(self):
        """Reset the emergency stop state"""
        self.stop_requested = False
        
    def cleanup(self):
        """Clean up GPIO resources"""
        GPIO.remove_event_detect(self.pin)