#!/usr/bin/env python3

import serial
import serial.tools.list_ports
import time
import binascii
import sys
import logging
import re
from struct import pack, unpack

# Set up logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# UBX Protocol constants
UBX_SYNC_CHAR_1 = 0xB5
UBX_SYNC_CHAR_2 = 0x62

# UBX message classes and IDs
CFG_TMODE3_CLASS = 0x06
CFG_TMODE3_ID = 0x71

# UBX-ACK messages for command verification
ACK_ACK_CLASS = 0x05
ACK_ACK_ID = 0x01
ACK_NAK_ID = 0x00

# UBX-CFG-PRT to configure ports
CFG_PRT_CLASS = 0x06
CFG_PRT_ID = 0x00

def calculate_checksum(message):
    """Calculate UBX checksum (8-bit Fletcher algorithm)"""
    ck_a = 0
    ck_b = 0
    
    for byte in message:
        ck_a += byte
        ck_a &= 0xFF
        ck_b += ck_a
        ck_b &= 0xFF
        
    return ck_a, ck_b

def create_ubx_message(msg_class, msg_id, payload=b''):
    """Create a UBX message with the specified class, ID and payload"""
    # Header: Class and ID
    header = pack('BB', msg_class, msg_id)
    
    # Calculate checksum over header and payload
    ck_a, ck_b = calculate_checksum(header + payload)
    
    # Assemble full message
    message = pack('BB', UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2)
    message += header
    message += pack('<H', len(payload))
    message += payload
    message += pack('BB', ck_a, ck_b)
    
    return message

def create_ubx_tmode3_poll():
    """Create a UBX-CFG-TMODE3 poll message (with no payload)"""
    return create_ubx_message(CFG_TMODE3_CLASS, CFG_TMODE3_ID)

def create_ubx_cfg_prt(port_id=1, protocol_mask=0x01):
    """Create a UBX-CFG-PRT message to enable UBX protocol
    
    port_id: 0=I2C, 1=UART1, 2=UART2, 3=USB, 4=SPI
    protocol_mask: bit0=UBX, bit1=NMEA, bit2=RTCM3
    """
    # For UART ports, we need to specify port configuration
    if port_id in [1, 2]:
        # Configure UART port for 9600 baud, 8N1, with UBX protocol
        payload = pack('<BBHIIHHHH', 
                      port_id,      # portID 
                      0,            # reserved
                      0,            # txReady
                      0x000008D0,   # mode (8N1)
                      9600,         # baudRate
                      protocol_mask,# inProtoMask
                      protocol_mask,# outProtoMask
                      0,            # reserved
                      0)            # reserved
    else:
        # For other ports, just set protocol mask
        payload = pack('<BB', port_id, 0)  # port ID and reserved
        payload += b'\x00' * 18           # Remaining bytes
        payload = bytearray(payload)
        payload[12] = protocol_mask       # inProtoMask
        payload[16] = protocol_mask       # outProtoMask
    
    return create_ubx_message(CFG_PRT_CLASS, CFG_PRT_ID, payload)

def wait_for_response(ser, expected_class, expected_id, timeout=2.0):
    """Wait for a specific UBX response message"""
    start_time = time.time()
    response = bytearray()
    packet_start = False
    sync1_found = False
    
    # For debugging, collect all bytes received
    all_bytes = bytearray()
    
    while time.time() - start_time < timeout:
        if ser.in_waiting > 0:
            byte = ser.read(1)[0]
            all_bytes.append(byte)
            
            # Look for UBX sync characters
            if not sync1_found and byte == UBX_SYNC_CHAR_1:
                sync1_found = True
                response = bytearray([byte])
            elif sync1_found and not packet_start and byte == UBX_SYNC_CHAR_2:
                packet_start = True
                response.append(byte)
            elif packet_start:
                response.append(byte)
                
                # If we have enough bytes to check message class and ID
                if len(response) >= 6:
                    msg_class = response[2]
                    msg_id = response[3]
                    payload_length = response[4] + (response[5] << 8)
                    
                    # If we have a complete message
                    if len(response) >= 6 + payload_length + 2:
                        # Check if this is the message we're looking for
                        if msg_class == expected_class and msg_id == expected_id:
                            return response
                        
                        # Otherwise, reset and keep looking
                        sync1_found = False
                        packet_start = False
                        response = bytearray()
                        
        time.sleep(0.01)
    
    # If we got here, we timed out
    if len(all_bytes) > 0:
        logger.debug(f"Received bytes: {binascii.hexlify(all_bytes)}")
        
        # Try to extract any NMEA messages in the received data
        try:
            nmea_text = all_bytes.decode('ascii', errors='ignore')
            nmea_messages = re.findall(r'\$GP[A-Z]{3}.*?(?:\*[0-9A-F]{2})?(?:\r\n|\n)', nmea_text)
            if nmea_messages:
                logger.info(f"Detected NMEA messages: {nmea_messages}")
                return "NMEA_DETECTED"
        except Exception as e:
            logger.debug(f"Error decoding NMEA: {e}")
    
    return None

def initialize_gps_for_ubx(ser):
    """Try to initialize the GPS to respond to UBX protocol commands"""
    # First, try to configure port for UBX protocol
    logger.info("Configuring port to use UBX protocol")
    
    # Try all common port IDs and protocol combinations
    port_ids = [1]  # UART1 is most common
    protocol_masks = [1, 3]  # UBX only, or UBX+NMEA
    
    for port_id in port_ids:
        for protocol_mask in protocol_masks:
            logger.debug(f"Trying port {port_id} with protocol mask {protocol_mask}")
            
            # Configure port
            ubx_prt_msg = create_ubx_cfg_prt(port_id, protocol_mask)
            ser.write(ubx_prt_msg)
            
            # Wait for ACK
            response = wait_for_response(ser, ACK_ACK_CLASS, ACK_ACK_ID, timeout=1.0)
            
            if response == "NMEA_DETECTED":
                logger.info("GPS is in NMEA mode, attempting to switch to UBX")
                return True
                
            if response:
                logger.info("Successfully configured port for UBX protocol")
                return True
    
    # If no response to UBX, try sending standard NMEA command
    # Some u-blox devices accept NMEA commands to switch to UBX
    logger.info("Trying NMEA command to switch to UBX mode")
    nmea_cmd = b"$PUBX,41,1,0003,0001,9600,0*16\r\n"
    ser.write(nmea_cmd)
    time.sleep(1.0)
    
    # Try a simple UBX command to see if we get a response
    ubx_poll = create_ubx_message(0x0A, 0x04)  # UBX-MON-VER
    ser.write(ubx_poll)
    response = wait_for_response(ser, 0x0A, 0x04, timeout=1.0)
    
    if response:
        logger.info("GPS is now responding to UBX commands")
        return True
    
    logger.warning("Failed to initialize GPS for UBX protocol")
    return False

def send_tmode3_poll_and_receive_response(ser):
    """Send TMODE3 poll and receive response"""
    message = create_ubx_tmode3_poll()
    
    # Flush input buffer
    ser.reset_input_buffer()
    
    # Send message
    logger.info(f"Sending UBX-CFG-TMODE3 poll command")
    logger.info(f"Poll message: {binascii.hexlify(message)}")
    ser.write(message)
    
    # Wait for response
    response = wait_for_response(ser, CFG_TMODE3_CLASS, CFG_TMODE3_ID, timeout=2.0)
    
    if not response:
        logger.error("Timeout waiting for TMODE3 response")
    elif response == "NMEA_DETECTED":
        logger.info("Device is in NMEA mode, need to switch to UBX first")
    
    return response

def parse_tmode3_response(response):
    """Parse the TMODE3 response according to the provided format"""
    if not response or response == "NMEA_DETECTED":
        return "No valid response received"
    
    # Extract header information
    msg_class = response[2]
    msg_id = response[3]
    payload_length = response[4] + (response[5] << 8)
    
    # Check if this is indeed a TMODE3 response
    if msg_class != CFG_TMODE3_CLASS or msg_id != CFG_TMODE3_ID:
        return f"Unexpected response: Class=0x{msg_class:02X}, ID=0x{msg_id:02X}"
    
    # Extract payload (skipping 6 bytes of header)
    payload = response[6:6+payload_length]
    
    # Check payload length
    if len(payload) != 40:  # TMODE3 payload should be 40 bytes
        return f"Invalid payload length: {len(payload)}, expected 40"
    
    try:
        # Parse the payload according to UBX-CFG-TMODE3 format
        version = payload[0]
        reserved1 = payload[1]
        flags = unpack('<H', payload[2:4])[0]
        
        # Extract mode from flags (bits 0-1)
        mode = flags & 0x03
        mode_descriptions = {
            0: "Disabled",
            1: "Survey-In",
            2: "Fixed Mode"
        }
        mode_desc = mode_descriptions.get(mode, f"Unknown ({mode})")
        
        # Extract other values
        ecefXOrLat = unpack('<i', payload[4:8])[0]
        ecefYOrLon = unpack('<i', payload[8:12])[0]
        ecefZOrAlt = unpack('<i', payload[12:16])[0]
        
        ecefXOrLatHP = payload[16]
        ecefYOrLonHP = payload[17]
        ecefZOrAltHP = payload[18]
        
        reserved2 = payload[19]
        fixedPosAcc = unpack('<I', payload[20:24])[0]
        svinMinDur = unpack('<I', payload[24:28])[0]
        svinAccLimit = unpack('<I', payload[28:32])[0]
        
        # Create result dictionary
        results = {
            "version": version,
            "flags": f"0x{flags:04X}",
            "mode": mode,
            "mode_description": mode_desc,
            "fixed_pos_acc_mm": fixedPosAcc * 0.1,
            "svin_min_dur_s": svinMinDur,
            "svin_acc_limit_mm": svinAccLimit * 0.1
        }
        
        # Check if position is in LLH or ECEF
        is_llh = (flags & 0x100) != 0
        results["coordinate_system"] = "LLH" if is_llh else "ECEF"
        
        if is_llh:
            # Convert to degrees and meters
            lat = (ecefXOrLat + ecefXOrLatHP * 1e-2) * 1e-7
            lon = (ecefYOrLon + ecefYOrLonHP * 1e-2) * 1e-7
            alt = (ecefZOrAlt + ecefZOrAltHP * 1e-2) * 1e-2  # Convert to meters
            
            results.update({
                "lat_deg": lat,
                "lon_deg": lon,
                "alt_m": alt
            })
        else:
            # ECEF coordinates in cm
            x = (ecefXOrLat + ecefXOrLatHP * 1e-2)
            y = (ecefYOrLon + ecefYOrLonHP * 1e-2)
            z = (ecefZOrAlt + ecefZOrAltHP * 1e-2)
            
            results.update({
                "ecef_x_cm": x,
                "ecef_y_cm": y,
                "ecef_z_cm": z
            })
        
        return results
        
    except Exception as e:
        logger.error(f"Error parsing TMODE3 payload: {e}")
        return f"Parse error: {e}"

def dump_raw_data(ser, duration=3.0):
    """Dump raw data from the serial port for debugging"""
    logger.info(f"Dumping raw data for {duration} seconds...")
    start_time = time.time()
    data = bytearray()
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            chunk = ser.read(ser.in_waiting)
            data.extend(chunk)
            
            # Try to print any ASCII/NMEA data
            try:
                ascii_data = chunk.decode('ascii', errors='ignore')
                if any(c in ascii_data for c in '$!'):  # NMEA messages start with $ or !
                    logger.info(f"NMEA data: {ascii_data.strip()}")
            except:
                pass
                
        time.sleep(0.1)
    
    logger.info(f"Raw data dump complete: {len(data)} bytes")
    logger.info(f"Hex: {binascii.hexlify(data)}")
    return data

def check_tmode_for_gps(port, baud_rates=None):
    """Check TMODE value for a GPS receiver on the given port"""
    if baud_rates is None:
        baud_rates = [9600, 38400, 115200]  # Common GPS baud rates
    
    for baud_rate in baud_rates:
        logger.info(f"Checking GPS on port {port} at {baud_rate} baud")
        
        try:
            # Open serial port
            ser = serial.Serial(port, baud_rate, timeout=1.0)
            
            # Add a small delay after opening
            time.sleep(0.5)
            
            # First, dump some raw data to see what's coming from the device
            raw_data = dump_raw_data(ser, 2.0)
            
            # Initialize GPS for UBX protocol
            if not initialize_gps_for_ubx(ser):
                logger.warning(f"Failed to initialize GPS on {port} at {baud_rate}")
                ser.close()
                continue
            
            # Send TMODE3 poll command
            response = send_tmode3_poll_and_receive_response(ser)
            
            # Parse and display response
            if response and response != "NMEA_DETECTED":
                logger.info(f"Raw response: {binascii.hexlify(response)}")
                parsed = parse_tmode3_response(response)
                logger.info(f"Parsed response: {parsed}")
                
                # Extract TMODE value
                if isinstance(parsed, dict) and "mode" in parsed:
                    tmode_value = parsed["mode"]
                    tmode_desc = parsed["mode_description"]
                    logger.info(f"TMODE value: {tmode_value} ({tmode_desc})")
                    ser.close()
                    return tmode_value
                else:
                    logger.warning("TMODE value not found in response")
            
            ser.close()
            
        except Exception as e:
            logger.error(f"Error checking GPS on {port} at {baud_rate} baud: {e}")
    
    return None

def find_and_check_gps_devices():
    """Find available serial ports and check for GPS devices"""
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        logger.error("No serial ports found")
        return
    
    logger.info(f"Found {len(ports)} serial ports")
    
    for port_info in ports:
        port = port_info.device
        logger.info(f"Checking port: {port} ({port_info.description})")
        
        tmode = check_tmode_for_gps(port)
        
        if tmode is not None:
            logger.info(f"GPS found on {port}, TMODE={tmode}")
        else:
            logger.info(f"No GPS or TMODE value found on {port}")

def check_specific_ports(port_list):
    """Check specific serial ports for GPS devices"""
    for port in port_list:
        # Check if port includes baud rate specification (port:baud)
        baud_rates = None
        if ":" in port:
            port, baud_str = port.split(":")
            try:
                baud_rates = [int(baud_str)]
                logger.info(f"Using specified baud rate: {baud_rates[0]}")
            except ValueError:
                logger.error(f"Invalid baud rate: {baud_str}, using default rates")
        
        tmode = check_tmode_for_gps(port, baud_rates)
        
        if tmode is not None:
            logger.info(f"GPS found on {port}, TMODE={tmode}")
        else:
            logger.info(f"No GPS or TMODE value found on {port}")

if __name__ == "__main__":
    logger.info("GPS TMODE checker starting")
    
    if len(sys.argv) > 1:
        # Use ports specified in command line
        ports = sys.argv[1:]
        logger.info(f"Checking specified ports: {ports}")
        check_specific_ports(ports)
    else:
        # Auto-discover ports
        logger.info("Auto-discovering serial ports")
        find_and_check_gps_devices()
        
    logger.info("GPS TMODE check completed")