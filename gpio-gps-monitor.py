#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/gpio_gps_monitor.py

import serial
import time
import argparse
import math
from datetime import datetime
import binascii

class UBXParser:
    """Parser for UBX protocol messages"""
    # UBX Constants
    UBX_SYNC_CHAR_1 = 0xB5
    UBX_SYNC_CHAR_2 = 0x62
    
    # Message Classes
    CLASS_NAV = 0x01
    
    # Message IDs for NAV class
    ID_NAV_RELPOSNED = 0x3C  # Relative Position NED
    ID_NAV_PVT = 0x07       # Position, Velocity, Time
    
    def __init__(self):
        self.buffer = bytearray()
        self.msg_class = None
        self.msg_id = None
        self.payload_length = 0
        self.payload = None
        self.checksum_a = 0
        self.checksum_b = 0
        
    def parse_byte(self, byte):
        """Process a single byte, return True if a complete message was found"""
        self.buffer.append(byte)
        
        # Keep buffer at a reasonable size if no valid messages found
        if len(self.buffer) > 1024:
            self.buffer = self.buffer[-512:]
            
        # Look for UBX header
        if len(self.buffer) >= 2:
            if self.buffer[0] == self.UBX_SYNC_CHAR_1 and self.buffer[1] == self.UBX_SYNC_CHAR_2:
                # We have a potential UBX message
                
                if len(self.buffer) < 6:
                    # Not enough bytes for header
                    return False
                
                # Extract message class and ID
                self.msg_class = self.buffer[2]
                self.msg_id = self.buffer[3]
                
                # Extract payload length (little endian)
                self.payload_length = self.buffer[4] + (self.buffer[5] << 8)
                
                # Check if we have the full message
                msg_length = 6 + self.payload_length + 2  # header + payload + checksum
                if len(self.buffer) >= msg_length:
                    # We have a complete message
                    self.payload = self.buffer[6:6+self.payload_length]
                    received_cka = self.buffer[6+self.payload_length]
                    received_ckb = self.buffer[6+self.payload_length+1]
                    
                    # Calculate checksum
                    cka = 0
                    ckb = 0
                    for i in range(2, 6+self.payload_length):
                        cka = (cka + self.buffer[i]) & 0xFF
                        ckb = (ckb + cka) & 0xFF
                    
                    # Verify checksum
                    if cka == received_cka and ckb == received_ckb:
                        # Valid message, clear buffer for next message
                        self.buffer = self.buffer[msg_length:]
                        return True
                    else:
                        # Invalid checksum, remove sync char and continue
                        self.buffer.pop(0)
                        return False
            else:
                # Not a UBX header, remove the first byte and continue
                self.buffer.pop(0)
                
        return False
    
    def parse_relposned(self):
        """Parse the RELPOSNED message to extract heading and other relevant data"""
        if self.msg_class == self.CLASS_NAV and self.msg_id == self.ID_NAV_RELPOSNED and self.payload:
            version = self.payload[0]
            
            # Extract flags - offset 4 in the message
            flags = self.payload[4] + (self.payload[5] << 8) + (self.payload[6] << 16) + (self.payload[7] << 24)
            
            # Check gnssFixOk flag
            gnss_fix_ok = (flags & 0x01) > 0
            
            # Check carrier solution status (bits 8-9)
            carr_soln = (flags >> 8) & 0x03  # 0=None, 1=Float, 2=Fixed
            
            # Extract relative position heading
            rel_pos_heading_valid = (flags & 0x00000400) > 0  # Bit 10
            
            # DEBUG: Print raw flags to diagnose issues
            print(f"DEBUG: RELPOSNED flags: 0x{flags:08X}")
            print(f"DEBUG: gnssFixOk: {gnss_fix_ok}, carr_soln: {carr_soln}, heading_valid: {rel_pos_heading_valid}")
            
            if rel_pos_heading_valid:
                # Extract relPosHeading (4 bytes at offset 36)
                heading_val = self.payload[36] + (self.payload[37] << 8) + \
                           (self.payload[38] << 16) + (self.payload[39] << 24)
                heading = heading_val / 100000.0  # Scale factor 1e-5
            else:
                heading = None
            
            # Extract relative position components
            n_mm = self.payload[8] + (self.payload[9] << 8) + (self.payload[10] << 16) + (self.payload[11] << 24)
            e_mm = self.payload[12] + (self.payload[13] << 8) + (self.payload[14] << 16) + (self.payload[15] << 24)
            d_mm = self.payload[16] + (self.payload[17] << 8) + (self.payload[18] << 16) + (self.payload[19] << 24)
            
            # Convert to meters
            rel_pos_n = n_mm / 1000.0
            rel_pos_e = e_mm / 1000.0
            rel_pos_d = d_mm / 1000.0
            
            # Calculate length of baseline
            baseline_length = math.sqrt(rel_pos_n**2 + rel_pos_e**2)
            
            # DEBUG: Print raw position components
            print(f"DEBUG: N: {rel_pos_n}m, E: {rel_pos_e}m, D: {rel_pos_d}m, Baseline: {baseline_length}m")
            
            # Extract accuracy estimates
            acc_n_mm = self.payload[24] + (self.payload[25] << 8) + (self.payload[26] << 16) + (self.payload[27] << 24)
            acc_e_mm = self.payload[28] + (self.payload[29] << 8) + (self.payload[30] << 16) + (self.payload[31] << 24)
            acc_d_mm = self.payload[32] + (self.payload[33] << 8) + (self.payload[34] << 16) + (self.payload[35] << 24)
            
            acc_n = acc_n_mm / 1000.0
            acc_e = acc_e_mm / 1000.0
            acc_d = acc_d_mm / 1000.0
            
            return {
                'heading': heading,
                'rel_pos_n': rel_pos_n,
                'rel_pos_e': rel_pos_e,
                'rel_pos_d': rel_pos_d,
                'baseline_length': baseline_length,
                'gnss_fix_ok': gnss_fix_ok,
                'carr_soln': carr_soln,
                'acc_n': acc_n, 
                'acc_e': acc_e,
                'acc_d': acc_d
            }
        return None
    
    def parse_pvt(self):
        """Parse the PVT message to extract position, velocity and time data"""
        if self.msg_class == self.CLASS_NAV and self.msg_id == self.ID_NAV_PVT and self.payload:
            # Extract positioning fix type
            fix_type = self.payload[20]
            
            # Extract latitude and longitude (degrees * 1e-7)
            lat_val = self.payload[28] + (self.payload[29] << 8) + \
                    (self.payload[30] << 16) + (self.payload[31] << 24)
            lon_val = self.payload[24] + (self.payload[25] << 8) + \
                    (self.payload[26] << 16) + (self.payload[27] << 24)
            
            # Convert to decimal degrees
            lat = lat_val * 1e-7
            lon = lon_val * 1e-7
            # Fix longitude value if needed
            if lon > 180:
                lon -= 360  # Convert to -180 to +180 range
            
            # Extract height above ellipsoid (mm)
            height_val = self.payload[32] + (self.payload[33] << 8) + \
                       (self.payload[34] << 16) + (self.payload[35] << 24)
            height = height_val / 1000.0  # Convert to meters
            
            # Extract ground speed (mm/s)
            speed_val = self.payload[60] + (self.payload[61] << 8) + \
                      (self.payload[62] << 16) + (self.payload[63] << 24)
            speed = speed_val / 1000.0  # Convert to m/s
            
            # Extract heading of motion (degrees * 1e-5)
            heading_val = self.payload[64] + (self.payload[65] << 8) + \
                        (self.payload[66] << 16) + (self.payload[67] << 24)
            heading_motion = heading_val * 1e-5
            
            return {
                'lat': lat,
                'lon': lon,
                'height': height,
                'fix_type': fix_type,
                'speed': speed,
                'heading_motion': heading_motion
            }
        return None

def monitor_gps_data(port="/dev/ttyS0", baud=460800):
    try:
        # 1. Open with appropriate timeout
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Connected to {port} at {baud} baud")
        
        # 2. Add a short wait after opening
        time.sleep(0.2)
        
        # 3. Use a buffer for accumulating partial messages
        buffer = ""
        
        while True:
            try:
                # 4. Read large chunks at once
                data = ser.read(1024)
                if data:
                    # 5. Decode with error handling
                    text = data.decode('ascii', errors='replace')
                    buffer += text
                    
                    # 6. Process complete messages
                    while '\n' in buffer:  # or '\r\n' depending on your GPS
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            # Process your line here
                            print(f"GPS: {line}")
                else:
                    # Small wait if no data to avoid CPU spinning
                    time.sleep(0.01)
                    
            except Exception as e:
                print(f"Error reading data: {e}")
                time.sleep(0.5)  # Wait before retry
                
    except KeyboardInterrupt:
        print("Monitoring stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Monitor GPS heading and position data on Raspberry Pi GPIO")
    parser.add_argument("--port", default="/dev/ttyS0", 
                        help="Hardware serial port (typically /dev/ttyS0 or /dev/serial0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    
    args = parser.parse_args()
    monitor_gps_data(args.port, args.baud)

