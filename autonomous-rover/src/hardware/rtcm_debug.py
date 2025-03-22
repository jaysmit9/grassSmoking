#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/hardware/rtcm_debug.py

import serial
import time
import binascii
import struct
from datetime import datetime

# RTCM message names
RTCM_MSG_TYPES = {
    1001: "GPS L1 RTK Observations",
    1002: "GPS L1 RTK Observations (Extended)",
    1003: "GPS L1/L2 RTK Observations",
    1004: "GPS L1/L2 RTK Observations (Extended)",
    1005: "Stationary RTK Reference Station ARP",
    1006: "Stationary RTK Reference Station ARP with Height",
    1007: "Antenna Descriptor",
    1008: "Antenna Descriptor & Serial Number",
    1009: "GLONASS L1 RTK Observations",
    1010: "GLONASS L1 RTK Observations (Extended)",
    1011: "GLONASS L1/L2 RTK Observations",
    1012: "GLONASS L1/L2 RTK Observations (Extended)",
    1019: "GPS Broadcast Ephemeris",
    1020: "GLONASS Broadcast Ephemeris",
    1033: "Receiver and Antenna Descriptors",
    1071: "GPS MSM1 (Minimal)",
    1072: "GPS MSM2 (Compact)",
    1073: "GPS MSM3 (Legacy)",
    1074: "GPS MSM4 (Full)",
    1075: "GPS MSM5 (Full plus bias)",
    1076: "GPS MSM6 (Full plus bias and STEC)",
    1077: "GPS MSM7 (Full plus ext bias)",
    1081: "GLONASS MSM1 (Minimal)",
    1082: "GLONASS MSM2 (Compact)",
    1083: "GLONASS MSM3 (Legacy)",
    1084: "GLONASS MSM4 (Full)",
    1085: "GLONASS MSM5 (Full plus bias)",
    1086: "GLONASS MSM6 (Full plus bias and STEC)",
    1087: "GLONASS MSM7 (Full plus ext bias)",
    1091: "Galileo MSM1 (Minimal)",
    1092: "Galileo MSM2 (Compact)",
    1093: "Galileo MSM3 (Legacy)",
    1094: "Galileo MSM4 (Full)",
    1095: "Galileo MSM5 (Full plus bias)",
    1096: "Galileo MSM6 (Full plus bias and STEC)",
    1097: "Galileo MSM7 (Full plus ext bias)",
    1101: "BeiDou MSM1", 
    1102: "BeiDou MSM2",
    1103: "BeiDou MSM3",
    1104: "BeiDou MSM4",
    1105: "BeiDou MSM5",
    1106: "BeiDou MSM6",
    1107: "BeiDou MSM7",
    1230: "GLONASS L1 and L2 Code-Phase Biases",
    4072: "u-blox Moving Base Data"
}

def calc_crc24q(data):
    """Calculate CRC-24Q checksum used by RTCM"""
    crc = 0
    for byte in data:
        crc ^= (byte << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF

def monitor_rtcm_messages(port="/dev/ttyS0", baud=460800):
    """Monitor RTCM messages on serial port"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Monitoring RTCM messages on {port} at {baud} baud...")
        print("Press Ctrl+C to stop\n")
        
        buffer = bytearray()
        stats = {
            "messages": 0,
            "last_reset": time.time(),
            "msg_types": {}
        }
        
        while True:
            # Read available data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                
                # Process buffer for RTCM messages (preamble is 0xD3)
                i = 0
                while i < len(buffer) - 3:  # Need at least preamble + 2 bytes
                    # Look for RTCM preamble
                    if buffer[i] == 0xD3:
                        # Check if we have at least the header to get message length
                        if i + 3 <= len(buffer):
                            # Extract message length (10 bits)
                            # First get the 16 bits containing the length
                            length_bits = buffer[i+1] << 8 | buffer[i+2]
                            # Length is the last 10 bits
                            msg_length = length_bits & 0x03FF
                            
                            # Ensure we have the full message plus 3-byte CRC
                            if len(buffer) >= i + 3 + msg_length + 3:
                                # We have a complete message
                                msg_data = buffer[i:i+3+msg_length]
                                crc_bytes = buffer[i+3+msg_length:i+3+msg_length+3]
                                received_crc = crc_bytes[0] << 16 | crc_bytes[1] << 8 | crc_bytes[2]
                                
                                # Calculate CRC
                                calculated_crc = calc_crc24q(msg_data)
                                
                                # Verify CRC
                                if received_crc == calculated_crc:
                                    # Extract message type (12 bits)
                                    msg_type_bits = buffer[i+3] << 4 | buffer[i+4] >> 4
                                    
                                    # Update stats
                                    stats["messages"] += 1
                                    if msg_type_bits in stats["msg_types"]:
                                        stats["msg_types"][msg_type_bits] += 1
                                    else:
                                        stats["msg_types"][msg_type_bits] = 1
                                    
                                    # Get message description
                                    msg_desc = RTCM_MSG_TYPES.get(msg_type_bits, "Unknown")
                                    
                                    # Print info
                                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                    print(f"{timestamp} - RTCM Type {msg_type_bits}: {msg_desc}")
                                    print(f"  Length: {msg_length} bytes")
                                    
                                    # For moving base message (4072), extract more details
                                    if msg_type_bits == 4072:
                                        subtype = buffer[i+5] & 0x0F
                                        print(f"  Subtype: {subtype}")
                                    
                                    # Show hex dump of first 16 bytes max
                                    max_display = min(16, msg_length)
                                    hex_data = " ".join([f"{b:02X}" for b in buffer[i+3:i+3+max_display]])
                                    print(f"  Data: {hex_data}{'...' if msg_length > 16 else ''}")
                                    print()
                                
                                # Move past this message
                                buffer = buffer[i+3+msg_length+3:]
                                i = 0
                                continue
                            
                    i += 1
                
                # Truncate buffer if it gets too large
                if len(buffer) > 8192:
                    buffer = buffer[-4096:]
            
            # Show stats every 5 seconds
            now = time.time()
            if now - stats["last_reset"] >= 5:
                print("\n--- RTCM Statistics ---")
                print(f"Total messages: {stats['messages']}")
                print("Message types:")
                for msg_type, count in sorted(stats["msg_types"].items()):
                    desc = RTCM_MSG_TYPES.get(msg_type, "Unknown")
                    print(f"  Type {msg_type}: {count} ({desc})")
                print("----------------------\n")
                stats["last_reset"] = now
            
            # Small delay
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Debug RTCM messages")
    parser.add_argument("--port", default="/dev/ttyS0", help="Serial port to monitor")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate")
    
    args = parser.parse_args()
    monitor_rtcm_messages(args.port, args.baud)