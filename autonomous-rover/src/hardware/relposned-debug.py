#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/relposned-debug.py

import serial
import time
import binascii
from datetime import datetime

def monitor_relposned(port="/dev/ttyS0", baud=460800):
    """Monitor RELPOSNED messages with enhanced debugging"""
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Monitoring RELPOSNED messages on {port} at {baud} baud...")
        
        buffer = bytearray()
        # UBX constants
        UBX_SYNC1 = 0xB5
        UBX_SYNC2 = 0x62
        UBX_CLASS_NAV = 0x01
        UBX_ID_RELPOSNED = 0x3C
        
        while True:
            # Read available data
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                buffer.extend(data)
                
                # Process buffer for UBX messages
                i = 0
                while i < len(buffer) - 8:
                    # Look for UBX header
                    if buffer[i] == UBX_SYNC1 and buffer[i+1] == UBX_SYNC2:
                        # Check if it's a RELPOSNED message
                        if buffer[i+2] == UBX_CLASS_NAV and buffer[i+3] == UBX_ID_RELPOSNED:
                            # Extract payload length
                            payload_len = buffer[i+4] + (buffer[i+5] << 8)
                            
                            # Check if we have the full message
                            if len(buffer) >= i + payload_len + 8:
                                # Complete message found
                                msg_buffer = buffer[i:i+payload_len+8]
                                payload = msg_buffer[6:6+payload_len]
                                
                                # Extract version
                                version = payload[0]
                                
                                # Extract flags (4 bytes)
                                flags = int.from_bytes(payload[4:8], byteorder='little')
                                
                                # Parse flags
                                gnss_fix_ok = bool(flags & 0x01)
                                diff_soln = bool(flags & 0x02)
                                rel_pos_valid = bool(flags & 0x04)
                                carr_soln = (flags >> 8) & 0x03  # 0=None, 1=Float, 2=Fixed
                                is_moving = bool(flags & 0x100)
                                ref_pos_miss = bool(flags & 0x200)
                                ref_obs_miss = bool(flags & 0x400)
                                rel_pos_heading_valid = bool(flags & 0x800)
                                rel_pos_norm = bool(flags & 0x1000)
                                
                                # Extract key fields
                                rel_pos_n = int.from_bytes(payload[8:12], byteorder='little', signed=True) / 100.0
                                rel_pos_e = int.from_bytes(payload[12:16], byteorder='little', signed=True) / 100.0
                                rel_pos_d = int.from_bytes(payload[16:20], byteorder='little', signed=True) / 100.0
                                
                                # Calculate baseline length
                                baseline = (rel_pos_n**2 + rel_pos_e**2 + rel_pos_d**2)**0.5
                                
                                # Get heading if available
                                heading = None
                                if payload_len >= 40:
                                    heading_raw = int.from_bytes(payload[36:40], byteorder='little')
                                    heading = heading_raw / 100000.0 if heading_raw != 0 else 0.0
                                
                                # Add this parsing to your RELPOSNED parser
                                heading_accuracy = int.from_bytes(payload[40:44], byteorder='little') / 100000.0 if payload_len >= 44 else float('nan')

                                # Print full debug info
                                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                                print(f"{timestamp} - RELPOSNED v{version}")
                                print(f"  FLAGS: 0x{flags:08X}")
                                print(f"  FIX: {gnss_fix_ok}, DIFF: {diff_soln}, VALID: {rel_pos_valid}, CARR: {carr_soln}")
                                print(f"  HEADING VALID: {rel_pos_heading_valid}, REF_MISS: {ref_obs_miss}")
                                print(f"  POS: N={rel_pos_n:.2f}cm E={rel_pos_e:.2f}cm D={rel_pos_d:.2f}cm")
                                print(f"  BASELINE: {baseline:.3f}m")
                                print(f"  HEADING: {heading:.2f}° (valid={rel_pos_heading_valid})")
                                print(f"  RAW PAYLOAD: {' '.join([f'{b:02X}' for b in payload[:40]])}")
                                print(f"  FLAGS details:")
                                print(f"    gnss_fix_ok: {gnss_fix_ok}")
                                print(f"    diff_soln: {diff_soln}")
                                print(f"    rel_pos_valid: {rel_pos_valid}")
                                print(f"    carr_soln: {carr_soln} ({['NONE', 'FLOAT', 'FIXED'][carr_soln] if carr_soln < 3 else 'INVALID'})")
                                print(f"    is_moving: {is_moving}")
                                print(f"    ref_pos_miss: {ref_pos_miss}")
                                print(f"    ref_obs_miss: {ref_obs_miss}")
                                print(f"    rel_pos_heading_valid: {rel_pos_heading_valid}")
                                print(f"  HEADING: {heading:.2f}° (±{heading_accuracy:.2f}°)")
                                print()
                                
                                # Remove processed message
                                buffer = buffer[i+payload_len+8:]
                                i = 0
                                continue
                    i += 1
                    
                # Truncate buffer if it gets too large
                if len(buffer) > 4096:
                    buffer = buffer[-2048:]
            
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
    parser = argparse.ArgumentParser(description="Debug RELPOSNED messages")
    parser.add_argument("--port", default="/dev/ttyS0", help="Serial port to monitor")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    
    args = parser.parse_args()
    monitor_relposned(args.port, args.baud)
