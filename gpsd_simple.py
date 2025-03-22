#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/gpsd_working.py

import gpsd
import time
import os
import signal
import sys
from datetime import datetime
import math

# Control flag
running = True

# Debug mode
DEBUG = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\nStopping GPS monitor...")
    running = False

def debug_print(msg):
    """Print debug messages if debug mode is enabled"""
    if DEBUG:
        print(f"[DEBUG] {msg}")

def get_fix_status(mode, error=None):
    """Determine fix quality from mode and error estimates"""
    if mode < 2:
        return "[NO FIX] 🔴"
    
    # Determine if we have RTK based on position error
    if error and 'x' in error and 'y' in error:
        epx, epy = error['x'], error['y']
        # Position error under 5cm typically indicates RTK fix
        if epx < 0.05 and epy < 0.05:
            return "[RTK FIXED] 🟢"
        # Position error under 50cm often indicates RTK float
        elif epx < 0.5 and epy < 0.5:
            return "[RTK FLOAT] 🟡"
    
    # Fall back to basic mode indicators
    if mode == 2:
        return "[2D FIX] 🟠"
    elif mode == 3:
        return "[3D FIX] 🔵"
    else:
        return f"[MODE {mode}]"

def main():
    global running
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting GPS monitor...")
    print("Press Ctrl+C to exit")
    
    try:
        # Connect to gpsd
        debug_print("Connecting to gpsd...")
        gpsd.connect()
        print("Connected to gpsd")
        
        # Main loop
        while running:
            try:
                # Get current GPS data
                packet = gpsd.get_current()
                
                # Clear screen
                os.system('clear')
                
                # Print header
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print("=" * 60)
                print(f"GPS MONITOR - {now}")
                print("=" * 60)
                
                # Get fix status
                mode = packet.mode
                error = getattr(packet, 'error', None)
                fix_status = get_fix_status(mode, error)
                
                print(f"\nSTATUS: {fix_status}")
                
                # Position data
                if mode >= 2:
                    print(f"\nPOSITION:")
                    print(f"  Latitude:  {packet.lat:.7f}°")
                    print(f"  Longitude: {packet.lon:.7f}°")
                    
                    # Try to get satellite info
                    sats = getattr(packet, 'sats', None)
                    if sats is not None and sats > 0:
                        print(f"  Satellites Used: {sats}")
                    
                    # Get position error estimates
                    if error and 'x' in error and 'y' in error:
                        print(f"  Precision: ±{error['x']:.3f}m E/W, ±{error['y']:.3f}m N/S")
                
                # 3D data
                if mode >= 3:
                    # Altitude
                    altitude = getattr(packet, 'alt', None)
                    if altitude is not None:
                        print(f"  Altitude:  {altitude:.2f}m")
                        # Vertical error if available
                        if error and 'v' in error:
                            print(f"  Vertical Precision: ±{error['v']:.3f}m")
                    
                    print(f"\nMOVEMENT:")
                    
                    # Speed
                    speed = getattr(packet, 'hspeed', None) 
                    if speed is None:
                        speed = getattr(packet, 'speed', None)
                        
                    if speed is not None:
                        print(f"  Speed: {speed:.2f} m/s ({speed*3.6:.1f} km/h)")
                        # Speed error
                        if error and 's' in error:
                            print(f"  Speed Precision: ±{error['s']:.3f} m/s")
                    
                    # Heading
                    track = getattr(packet, 'track', None)
                    if track is not None:
                        print(f"  Heading: {track:.1f}°")
                        # Track error
                        if error and 't' in error:
                            print(f"  Heading Precision: ±{error['t']:.3f}°")
                    
                    # Vertical speed
                    climb = getattr(packet, 'climb', None)
                    if climb is not None:
                        print(f"  Vertical Speed: {climb:.2f} m/s")
                        # Climb error
                        if error and 'c' in error:
                            print(f"  Vertical Speed Precision: ±{error['c']:.3f} m/s")
                
                # Time data
                gps_time = getattr(packet, 'time', None)
                if gps_time:
                    print(f"\nTIME:")
                    print(f"  GPS Time: {gps_time}")
                
                # Raw data for debugging
                print("\nRAW DATA:")
                for attr_name in dir(packet):
                    # Skip private attributes and methods
                    if not attr_name.startswith('_') and not callable(getattr(packet, attr_name)):
                        attr_value = getattr(packet, attr_name)
                        print(f"  {attr_name}: {attr_value}")
                
                time.sleep(1.0)
                
            except Exception as e:
                print(f"Error: {e}")
                import traceback
                traceback.print_exc()
                time.sleep(1)
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("GPS monitor stopped.")

if __name__ == "__main__":
    main()