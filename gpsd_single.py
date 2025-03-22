#!/usr/bin/env python3

import gpsd
import time
import os
import signal
import sys
from datetime import datetime
import math
import traceback

# Control flag
running = True

# Device tracking
devices = {
    "device1": {
        "name": "GPS 1",
        "data": None,
        "timestamp": 0,
        "position": None,
        "path": None
    },
    "device2": {
        "name": "GPS 2",
        "data": None,
        "timestamp": 0,
        "position": None,
        "path": None
    }
}

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

def get_fix_status(mode, epx=None, epy=None):
    """Determine fix quality from mode and error estimates"""
    if mode < 2:
        return "[NO FIX] 🔴"
    
    # Determine if we have RTK based on position error
    if epx is not None and epy is not None:
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

def update_device_data(packet):
    """Update data for the current device"""
    global devices
    
    try:
        # Get device path
        device_path = getattr(packet, 'device', None)
        if not device_path:
            debug_print("Packet has no device path")
            return
            
        debug_print(f"Processing data from device: {device_path}")
        
        # Determine which device slot to use
        target_device = None
        
        # Check if this device matches one we're already tracking
        for key, device_data in devices.items():
            if "path" in device_data and device_data["path"] == device_path:
                target_device = key
                debug_print(f"Found existing device: {key}")
                break
        
        # If not found, assign it to the first empty slot
        if not target_device:
            for key, device_data in devices.items():
                if device_data["path"] is None:
                    target_device = key
                    devices[key]["path"] = device_path
                    devices[key]["name"] = f"GPS ({device_path.split('/')[-1]})"
                    debug_print(f"Assigned to new slot: {key}")
                    break
        
        # If still not assigned, check timestamps and replace oldest data
        if not target_device:
            oldest_time = float('inf')
            oldest_device = None
            for key, device_data in devices.items():
                if device_data["timestamp"] < oldest_time:
                    oldest_time = device_data["timestamp"]
                    oldest_device = key
            
            if oldest_device:
                target_device = oldest_device
                devices[target_device]["path"] = device_path
                devices[target_device]["name"] = f"GPS ({device_path.split('/')[-1]})"
                debug_print(f"Replaced oldest device: {oldest_device}")
        
        # Update device data
        if target_device:
            # Basic data
            devices[target_device]["timestamp"] = time.time()
            devices[target_device]["data"] = packet
            
            # Save position if available
            if packet.mode >= 2:
                devices[target_device]["position"] = (packet.lat, packet.lon)
            
            debug_print(f"Updated {target_device} with mode {packet.mode}")
            
    except Exception as e:
        debug_print(f"Error in update_device_data: {e}")
        traceback.print_exc()

def calculate_distance():
    """Calculate distance between GPS units if both positions are available"""
    pos1 = devices["device1"].get("position")
    pos2 = devices["device2"].get("position")
    
    if pos1 and pos2:
        # Haversine formula
        lat1, lon1 = map(math.radians, pos1)
        lat2, lon2 = map(math.radians, pos2)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        # Earth radius in meters
        r = 6371000
        
        distance = r * c
        
        # Calculate bearing
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360
        
        return distance, bearing
    
    return None, None

def display_device_data(device_id):
    """Display data for a specific device"""
    device = devices[device_id]
    
    # Check if we have data for this device
    if "data" not in device or not device["data"]:
        print(f"{device.get('name', device_id)}: No data available")
        return
        
    packet = device["data"]
    
    print(f"--- {device.get('name', device_id)} ---")
    print(f"Device: {getattr(packet, 'device', 'Unknown')}")
    
    # Get fix status
    mode = packet.mode
    epx = getattr(packet, 'epx', None)
    epy = getattr(packet, 'epy', None)
    
    fix_status = get_fix_status(mode, epx, epy)
    print(f"Status: {fix_status}")
    
    # Position data
    if mode >= 2:
        try:
            print(f"Position: {packet.lat:.7f}, {packet.lon:.7f}")
            
            # Satellites
            try:
                sats = packet.sats
                sats_used = sum(1 for sat in sats if sat.used)
                print(f"Satellites: {sats_used}/{len(sats)}")
            except:
                pass
        except:
            print("Position data error")
    else:
        print("Position: No fix")
    
    # Additional data for 3D fix
    if mode >= 3:
        altitude = getattr(packet, 'alt', None)
        if altitude is not None:
            print(f"Altitude: {altitude:.2f}m")
            
        speed = getattr(packet, 'speed', None)
        if speed is not None:
            print(f"Speed: {speed:.2f} m/s ({speed*3.6:.1f} km/h)")
            
        track = getattr(packet, 'track', None)
        if track is not None:
            print(f"Heading: {track:.1f}°")
    
    # Precision data
    print("Precision:")
    if epx is not None and epy is not None:
        print(f"  Horizontal: ±{epx:.3f}m, ±{epy:.3f}m")
    
    epv = getattr(packet, 'epv', None)
    if epv is not None:
        print(f"  Vertical: ±{epv:.3f}m")
    
    # Data age
    age = time.time() - device["timestamp"]
    print(f"Data age: {age:.1f}s")

def get_gpsd_device_list():
    """Try to get list of devices from gpsd"""
    try:
        # This isn't typically available in the Python gpsd library, but let's try
        debug_print("Attempting to enumerate gpsd devices...")
        devices_packet = gpsd.get_current()
        if hasattr(devices_packet, 'devices'):
            return devices_packet.devices
    except:
        pass
    
    return []

def main():
    global running
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting dual GPS monitor...")
    print("Press Ctrl+C to exit")
    
    try:
        # Connect to gpsd
        debug_print("Connecting to gpsd...")
        gpsd.connect()
        
        print("Connected to gpsd")
        
        # Try to enumerate devices
        device_list = get_gpsd_device_list()
        if device_list:
            debug_print(f"Found devices: {device_list}")
        
        # Check if gpsd reports any data
        try:
            debug_print("Attempting to get initial GPS data...")
            initial_packet = gpsd.get_current()
            debug_print(f"Initial GPS data received: {initial_packet.__dict__}")
        except Exception as e:
            debug_print(f"Error getting initial GPS data: {e}")
        
        # Main loop
        while running:
            try:
                # Get current GPS packet
                debug_print("Polling for GPS data...")
                packet = gpsd.get_current()
                debug_print(f"Received packet of class: {packet.__class__.__name__}")
                
                # Update appropriate device data
                update_device_data(packet)
                
                if not DEBUG:
                    # Clear screen (skip in debug mode)
                    os.system('cls' if os.name == 'nt' else 'clear')
                
                # Print header
                now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                print("=" * 60)
                print(f"DUAL GPS MONITOR - {now}")
                print("=" * 60)
                
                # Print gpsd device list if available
                if device_list:
                    print(f"GPSD Devices: {', '.join(device_list)}")
                
                # Display data for each device
                print("\n")
                display_device_data("device1")
                
                print("\n")
                display_device_data("device2")
                
                # Calculate and show distance between GPS units
                distance, bearing = calculate_distance()
                if distance is not None:
                    print("\n=== GPS SEPARATION ===")
                    print(f"Distance: {distance:.2f}m")
                    print(f"Bearing: {bearing:.1f}°")
                
                # Pause
                time.sleep(1.0)  # Increased for better readability in debug mode
                
            except Exception as e:
                print(f"Error in main loop: {str(e)}")
                traceback.print_exc()
                time.sleep(1)
                
    except Exception as e:
        print(f"Initialization error: {str(e)}")
        traceback.print_exc()
    finally:
        print("GPS monitor stopped.")

if __name__ == "__main__":
    main()