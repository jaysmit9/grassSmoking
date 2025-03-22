#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/multi_gps_monitor.py

import gpsd
import time
import os
import signal
import sys
from datetime import datetime
import subprocess
import json

# Control flag
running = True

# Store data from each device
gps_data = {
    "device1": {
        "name": "GPS 1",
        "last_data": None,
        "timestamp": 0
    },
    "device2": {
        "name": "GPS 2",
        "last_data": None,
        "timestamp": 0
    }
}

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\nStopping GPS monitor...")
    running = False

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

def get_gpsd_devices():
    """Get list of devices connected to gpsd using gpspipe"""
    try:
        result = subprocess.run(
            ["gpspipe", "-w", "-n", "5"], 
            capture_output=True, 
            text=True,
            timeout=3
        )
        
        for line in result.stdout.splitlines():
            try:
                data = json.loads(line)
                if data.get("class") == "DEVICES" and "devices" in data:
                    return data["devices"]
            except json.JSONDecodeError:
                pass
    except Exception as e:
        print(f"Error getting gpsd devices: {e}")
    
    return []

def try_switch_device(device_path):
    """Attempt to make gpsd switch to a specific device"""
    try:
        # This is a theoretic approach - may not work with all gpsd setups
        subprocess.run(
            ["gpsctl", "-s", device_path],
            capture_output=True,
            text=True,
            timeout=2
        )
        time.sleep(0.5)  # Give gpsd time to switch
        return True
    except:
        return False

def display_gps_data(name, data, timestamp):
    """Display formatted GPS data"""
    if data is None:
        print(f"{name}: No data available")
        return
        
    age = time.time() - timestamp
    print(f"=== {name} (data age: {age:.1f}s) ===")
    
    # Basic status
    mode = data.mode
    error = getattr(data, 'error', None)
    fix_status = get_fix_status(mode, error)
    print(f"Status: {fix_status}")
    
    # Position data
    if mode >= 2:
        print(f"Position: {data.lat:.7f}, {data.lon:.7f}")
        
        # Get position error estimates
        if error and 'x' in error and 'y' in error:
            print(f"Precision: ±{error['x']:.3f}m E/W, ±{error['y']:.3f}m N/S")
    
    # 3D data
    if mode >= 3:
        # Altitude
        altitude = getattr(data, 'alt', None)
        if altitude is not None:
            print(f"Altitude: {altitude:.2f}m")
        
        # Speed
        speed = getattr(data, 'hspeed', None) 
        if speed is None:
            speed = getattr(data, 'speed', None)
            
        if speed is not None:
            print(f"Speed: {speed:.2f} m/s ({speed*3.6:.1f} km/h)")
        
        # Heading
        track = getattr(data, 'track', None)
        if track is not None:
            print(f"Heading: {track:.1f}°")

def main():
    global running, gps_data
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting multiple GPS monitor...")
    print("Press Ctrl+C to exit")
    
    # Get list of available devices
    devices = get_gpsd_devices()
    if devices:
        print(f"Found {len(devices)} GPS devices:")
        for i, device in enumerate(devices):
            print(f"  {i+1}. {device.get('path', 'Unknown')}")
            
            # Assign devices to our data structure
            if i == 0 and "path" in device:
                gps_data["device1"]["name"] = f"GPS 1 ({device['path'].split('/')[-1]})"
                gps_data["device1"]["path"] = device["path"]
            elif i == 1 and "path" in device:
                gps_data["device2"]["name"] = f"GPS 2 ({device['path'].split('/')[-1]})"
                gps_data["device2"]["path"] = device["path"]
    else:
        print("No GPS devices found. Will use default device.")
    
    # Connect to gpsd
    gpsd.connect()
    print("Connected to gpsd")
    
    # Time tracking for device switching
    last_switch = 0
    current_device = "device1"
    
    try:
        # Main loop
        while running:
            try:
                now = time.time()
                
                # Try to switch devices periodically
                if len(devices) > 1 and now - last_switch > 3:
                    # Alternate between devices
                    if current_device == "device1":
                        current_device = "device2"
                        if "path" in gps_data["device2"]:
                            try_switch_device(gps_data["device2"]["path"])
                    else:
                        current_device = "device1"
                        if "path" in gps_data["device1"]:
                            try_switch_device(gps_data["device1"]["path"])
                            
                    last_switch = now
                
                # Get current GPS data
                packet = gpsd.get_current()
                
                # Store data with the current device
                gps_data[current_device]["last_data"] = packet
                gps_data[current_device]["timestamp"] = now
                
                # Clear screen
                os.system('clear')
                
                # Print header
                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print("=" * 60)
                print(f"DUAL GPS MONITOR - {now_str}")
                print("=" * 60)
                
                # Display data for both devices
                print("\n")
                display_gps_data(
                    gps_data["device1"]["name"], 
                    gps_data["device1"]["last_data"],
                    gps_data["device1"]["timestamp"]
                )
                
                print("\n")
                display_gps_data(
                    gps_data["device2"]["name"], 
                    gps_data["device2"]["last_data"],
                    gps_data["device2"]["timestamp"]
                )
                
                # Pause
                time.sleep(0.5)
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(1)
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("GPS monitor stopped.")

if __name__ == "__main__":
    main()