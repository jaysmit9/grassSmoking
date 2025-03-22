#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/configure_rover.py

import serial
import time
import argparse
import binascii

def calculate_ubx_checksum(message):
    """Calculate checksum for UBX message"""
    ck_a = 0
    ck_b = 0
    for i in range(2, len(message)):
        ck_a = (ck_a + message[i]) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def create_ubx_message(msg_class, msg_id, payload):
    """Create a complete UBX message with checksum"""
    message = bytearray([0xB5, 0x62, msg_class, msg_id, len(payload) & 0xFF, (len(payload) >> 8) & 0xFF])
    message.extend(payload)
    ck_a, ck_b = calculate_ubx_checksum(message)
    message.extend([ck_a, ck_b])
    return message

def send_config_message(ser, key_id, value, layer=1):
    """Send a UBX-CFG-VALSET message to set a configuration value"""
    # UBX-CFG-VALSET message
    # Layer: 1=RAM, 2=BBR, 4=Flash
    header = bytearray([0, layer, 0, 0])  # version, layer, reserved
    payload = bytearray(header)
    payload.extend(key_id)
    payload.extend(value)
    
    message = create_ubx_message(0x06, 0x8A, payload)
    
    # Convert to hex string for display
    hex_str = " ".join([f"{b:02X}" for b in message])
    print(f"Sending: {hex_str}")
    
    # Send the message
    ser.write(message)
    time.sleep(0.1)  # Give the device time to process

def send_poll_message(ser, key_id=None):
    """Send a UBX-CFG-VALGET message to query configuration values"""
    # UBX-CFG-VALGET message
    if key_id:
        # Poll specific key
        header = bytearray([0, 0, 0, 0])  # version, layer (0=RAM), reserved
        payload = bytearray(header)
        payload.extend(key_id)
    else:
        # Poll all keys in NAVHPG section
        header = bytearray([0, 0, 0, 0])  # version, layer (0=RAM), reserved
        payload = bytearray(header)
        payload.extend([0x2E, 0x00, 0x11, 0x00])  # CFG-NAVHPG section
    
    message = create_ubx_message(0x06, 0x8B, payload)
    
    # Convert to hex string for display
    hex_str = " ".join([f"{b:02X}" for b in message])
    print(f"Polling: {hex_str}")
    
    # Send the message
    ser.write(message)
    
def configure_rover(port, baud=115200):
    """Configure rover GPS for heading operation"""
    try:
        # Open serial connection
        ser = serial.Serial(port, baud, timeout=1.0)
        print(f"Connected to {port} at {baud} baud")
        
        # Configuration key IDs and values
        configs = [
            # Basic RTK configuration
            ([0x11, 0x00, 0x11, 0x10], [0x03, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-DGNSSMODE = 3 (RTK)
            ([0x11, 0x00, 0x11, 0x01], [0x04, 0x00, 0x00, 0x00]),  # CFG-NAVSPG-DYNMODEL = 4 (automotive)
            
            # Heading configuration
            ([0x2E, 0x00, 0x11, 0x20], [0x01, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-HEADINGENA = 1 (enable)
            ([0x2E, 0x00, 0x11, 0x21], [0x01, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-DOAUTOCONFIG = 1 (enable)
            
            # Baseline configuration
            
            ([0x2E, 0x00, 0x11, 0x06], [0x29, 0x03, 0x00, 0x00]),  # CFG-NAVHPG-BASELEN = 809 (0.81m in cm)
            ([0x2E, 0x00, 0x11, 0x07], [0x00, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-BASELNSIG = 0
            ([0x2E, 0x00, 0x11, 0x09], [0x00, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-BASEFIXED = 0 (moving base)
            ([0x2E, 0x00, 0x11, 0x30], [0x00, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-MBASHAFT = 0
            
            # Message output configuration  
            ([0x91, 0x00, 0x20, 0x01], [0x01, 0x00, 0x00, 0x00]),  # CFG-MSGOUT-UBX_NAV_RELPOSNED_USB = 1
            
            # Try different antenna configuration
            ([0x2E, 0x00, 0x11, 0x35], [0x01, 0x00, 0x00, 0x00]),  # CFG-NAVHPG-MBANTENNASEL = 1 (2 antennas)
            ([0x2E, 0x00, 0x11, 0x36], [0x51, 0x03, 0x00, 0x00]),  # CFG-NAVHPG-MBANTENNAOFFX = baseline in mm (81cm = 810mm)
        ]
        
        # Send each configuration message
        print("Sending configuration messages...")
        for key_id, value in configs:
            send_config_message(ser, key_id, value)
            
        # Save configuration to flash
        print("Saving configuration to flash...")
        save_payload = bytearray([0, 7, 0, 0])  # Save to RAM, BBR, and Flash
        save_msg = create_ubx_message(0x06, 0x09, save_payload)
        ser.write(save_msg)
        time.sleep(1.0)
        
        # Poll to verify settings
        print("\nPolling configuration to verify...")
        send_poll_message(ser)
        
        # Wait for and print response
        print("Waiting for response...")
        time.sleep(1.0)
        
        # Read response
        read_count = 0
        timeout = time.time() + 5.0
        while time.time() < timeout:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                hex_data = " ".join([f"{b:02X}" for b in data])
                print(f"Received: {hex_data}")
                read_count += len(data)
            time.sleep(0.1)
            
        print(f"Read {read_count} bytes")
        print("Configuration complete!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Configure ZED-F9P for heading")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    
    args = parser.parse_args()
    configure_rover(args.port, args.baud)