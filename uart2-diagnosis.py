#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/uart2-diagnostic.py

import serial
import time
import binascii
from datetime import datetime

def monitor_mixed_data(port="/dev/ttyACM0", baud=115200):
    """Monitor both RTCM and UBX messages on the same port"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Monitoring UART for mixed protocol data ({port} at {baud} baud)...")
        
        buffer = bytearray()
        rtcm_count = 0
        ubx_count = 0
        unknown_count = 0
        last_status_time = time.time()
        
        # Constants
        RTCM3_PREAMBLE = 0xD3
        UBX_SYNC1 = 0xB5
        UBX_SYNC2 = 0x62
        
        while True:
            # Read available data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
            
            # Process buffer
            while len(buffer) > 6:  # Need enough bytes for header detection
                # Check for RTCM3 message
                if buffer[0] == RTCM3_PREAMBLE:
                    if len(buffer) >= 3:
                        length = ((buffer[1] & 0x03) << 8) | buffer[2]
                        
                        if len(buffer) >= length + 6:
                            # RTCM message found
                            msg_type = (buffer[3] << 4) | ((buffer[4] & 0xF0) >> 4)
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            print(f"{timestamp} - RTCM: Type {msg_type}, Length {length}")
                            
                            rtcm_count += 1
                            buffer = buffer[length + 6:]
                            continue
                    # Need more data for RTCM
                    break
                
                # Check for UBX message
                elif buffer[0] == UBX_SYNC1 and buffer[1] == UBX_SYNC2:
                    if len(buffer) >= 6:
                        msg_class = buffer[2]
                        msg_id = buffer[3]
                        payload_len = buffer[4] + (buffer[5] << 8)
                        
                        if len(buffer) >= payload_len + 8:
                            # UBX message found
                            msg_name = "UNKNOWN"
                            if msg_class == 0x01:  # NAV class
                                if msg_id == 0x3C:
                                    msg_name = "NAV-RELPOSNED"
                                elif msg_id == 0x07:
                                    msg_name = "NAV-PVT"
                            
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            print(f"{timestamp} - UBX: Class {msg_class:02X}, ID {msg_id:02X} ({msg_name}), Length {payload_len}")
                            
                            # If it's a RELPOSNED message, parse it for detailed info
                            if msg_class == 0x01 and msg_id == 0x3C and payload_len >= 40:
                                # Extract RELPOSNED fields
                                relpos_payload = buffer[6:6+payload_len]
                                
                                # Extract flags
                                flags = int.from_bytes(relpos_payload[4:8], byteorder='little')
                                gnss_fix_ok = (flags & 0x01) > 0
                                rel_pos_valid = (flags & 0x02) > 0
                                carr_soln = (flags >> 8) & 0x03  # 0=none, 1=float, 2=fixed
                                
                                # Extract heading if valid
                                rel_pos_heading_valid = (flags & 0x400) > 0
                                if rel_pos_heading_valid and payload_len >= 40:
                                    heading = int.from_bytes(relpos_payload[36:40], byteorder='little') / 100000.0
                                    print(f"  --> Heading: {heading:.1f}°, Solution: {carr_soln}")
                                else:
                                    print(f"  --> Heading not valid, Solution: {carr_soln}")
                            
                            ubx_count += 1
                            buffer = buffer[payload_len + 8:]
                            continue
                    # Need more data for UBX
                    break
                
                # Unknown byte, skip it
                else:
                    unknown_count += 1
                    buffer.pop(0)
            
            # Print statistics every 10 seconds
            if time.time() - last_status_time > 10:
                print("\n--- Protocol Statistics ---")
                print(f"RTCM messages: {rtcm_count}")
                print(f"UBX messages: {ubx_count}")
                print(f"Unknown bytes skipped: {unknown_count}")
                print("------------------------\n")
                last_status_time = time.time()
                
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
    parser = argparse.ArgumentParser(description="Monitor mixed protocol data on serial port")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port to monitor")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    
    args = parser.parse_args()
    monitor_mixed_data(args.port, args.baud)