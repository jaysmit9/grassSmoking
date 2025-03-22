#!/usr/bin/env python3
# filepath: /home/jay/projects/smokingGrass/gpio_rtcm_monitor.py

import serial
import time
import argparse
from datetime import datetime

# RTCM3 message constants
RTCM3_PREAMBLE = 0xD3

def monitor_gpio_rtcm(port="/dev/ttyS0", baud=115200):
    """Monitor RTCM messages on the Raspberry Pi GPIO UART pins"""
    try:
        # Open serial connection to Pi's hardware UART
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Monitoring RTCM3 messages on GPIO UART ({port}) at {baud} baud...")
        
        buffer = bytearray()
        message_count = 0
        message_types = {}
        last_status_time = time.time()
        
        while True:
            # Read available data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
            
            # Process buffer for RTCM messages
            while len(buffer) > 3:  # Need at least preamble + length
                # Look for RTCM preamble
                if buffer[0] == RTCM3_PREAMBLE:
                    # Extract message length (10 bits from bytes 1-2)
                    length = ((buffer[1] & 0x03) << 8) | buffer[2]
                    
                    # Check if we have the complete message
                    if len(buffer) >= length + 6:  # preamble + length + data + 3 byte CRC
                        # Extract message type (first 12 bits after preamble+length)
                        if length >= 2:
                            msg_type = (buffer[3] << 4) | ((buffer[4] & 0xF0) >> 4)
                            
                            # Update statistics
                            message_count += 1
                            if msg_type in message_types:
                                message_types[msg_type] += 1
                            else:
                                message_types[msg_type] = 1
                                
                            # Print message info
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            print(f"{timestamp} - RTCM message: Type {msg_type}, Length {length}")
                            
                        # Remove processed message from buffer
                        buffer = buffer[length + 6:]
                    else:
                        # Wait for more data
                        break
                else:
                    # Not an RTCM message, remove byte and continue
                    buffer.pop(0)
            
            # Print statistics every 10 seconds
            if time.time() - last_status_time > 10:
                print("\n--- RTCM Statistics ---")
                print(f"RTCM messages detected: {message_count}")
                print("Message types:")
                for msg_type, count in sorted(message_types.items()):
                    print(f"  Type {msg_type}: {count} messages")
                print("----------------------\n")
                last_status_time = time.time()
                
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor RTCM3 messages on Raspberry Pi GPIO")
    parser.add_argument("--port", default="/dev/ttyS0", 
                        help="Hardware serial port (typically /dev/ttyS0 or /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    
    args = parser.parse_args()
    monitor_gpio_rtcm(args.port, args.baud)