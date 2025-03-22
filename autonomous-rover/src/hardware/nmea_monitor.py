#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/hardware/direct_serial_monitor.py

import serial
import time
import os
import sys
import glob
from datetime import datetime

# ANSI color codes
COLOR_RESET = "\033[0m"
COLOR_GREEN = "\033[32m"
COLOR_YELLOW = "\033[33m"
COLOR_BLUE = "\033[34m"
COLOR_MAGENTA = "\033[35m"
COLOR_CYAN = "\033[36m"
COLOR_RED = "\033[31m"

def find_serial_ports():
    """Find available serial ports"""
    if sys.platform.startswith('linux'):
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    elif sys.platform.startswith('win'):
        # Windows ports
        return ['COM%s' % (i + 1) for i in range(256)]
    else:
        return glob.glob('/dev/tty.*') + glob.glob('/dev/cu.*')

def extract_gga_info(sentence):
    """Extract information from GGA sentence including RTK status"""
    parts = sentence.split(',')
    if len(parts) < 15:
        return None
        
    try:
        quality = int(parts[6]) if parts[6] else 0
        quality_text = {
            0: "NO FIX",
            1: "GPS FIX",
            2: "DGPS FIX", 
            4: "RTK FIXED",
            5: "RTK FLOAT",
            6: "ESTIMATED"
        }.get(quality, f"UNKNOWN({quality})")
        
        satellites = int(parts[7]) if parts[7] else 0
        
        return {
            "quality": quality,
            "quality_text": quality_text,
            "satellites": satellites
        }
    except Exception:
        return None

def colorize_sentence(sentence):
    """Colorize NMEA sentence based on type"""
    if sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA'):
        return f"{COLOR_GREEN}{sentence}{COLOR_RESET}"
    elif sentence.startswith('$GPRMC') or sentence.startswith('$GNRMC'):
        return f"{COLOR_BLUE}{sentence}{COLOR_RESET}"
    elif sentence.startswith('$GPGSV') or sentence.startswith('$GNGSV'):
        return f"{COLOR_YELLOW}{sentence}{COLOR_RESET}"
    elif sentence.startswith('$GPGSA') or sentence.startswith('$GNGSA'):
        return f"{COLOR_MAGENTA}{sentence}{COLOR_RESET}"
    elif sentence.startswith('$GPVTG') or sentence.startswith('$GNVTG'):
        return f"{COLOR_CYAN}{sentence}{COLOR_RESET}"
    else:
        return sentence

def monitor_serial_port(port, baud_rate=460800):
    """Monitor NMEA sentences directly from serial port"""
    try:
        with serial.Serial(port, baud_rate, timeout=0.1) as ser:
            print(f"Connected to {port} at {baud_rate} baud")
            print(f"Press Ctrl+C to exit")
            
            # Buffer for incomplete sentences
            buffer = ""
            
            # Counts of sentence types
            sentence_counts = {}
            
            # Last RTK status
            last_rtk_status = None
            last_clear = time.time()
            
            # Loop and read data
            while True:
                try:
                    # Read from serial port
                    data = ser.read(1024)
                    if data:
                        # Decode and add to buffer
                        text = data.decode('ascii', errors='replace')
                        buffer += text
                        
                        # Process complete sentences
                        while '\r\n' in buffer:
                            sentence, buffer = buffer.split('\r\n', 1)
                            
                            # Only process valid NMEA sentences
                            if sentence.startswith('$'):
                                # Update counts
                                sentence_type = sentence[:6]
                                sentence_counts[sentence_type] = sentence_counts.get(sentence_type, 0) + 1
                                
                                # Check for GGA (RTK status)
                                rtk_info = None
                                if sentence.startswith('$GPGGA') or sentence.startswith('$GNGGA'):
                                    rtk_info = extract_gga_info(sentence)
                                    if rtk_info:
                                        last_rtk_status = rtk_info
                                
                                # Display with RTK status if available
                                if rtk_info and rtk_info['quality'] > 0:
                                    quality_color = COLOR_RED
                                    if rtk_info['quality'] == 4:
                                        quality_color = COLOR_GREEN
                                    elif rtk_info['quality'] == 5:
                                        quality_color = COLOR_YELLOW
                                    elif rtk_info['quality'] == 2:
                                        quality_color = COLOR_BLUE
                                        
                                    quality_display = f"{quality_color}{rtk_info['quality_text']}({rtk_info['satellites']} sats){COLOR_RESET}"
                                    print(f"[{quality_display}] {colorize_sentence(sentence)}")
                                else:
                                    print(colorize_sentence(sentence))
                    
                    # Clear screen periodically
                    now = time.time()
                    if now - last_clear > 30:
                        os.system('clear')
                        last_clear = now
                        
                        # Print header with stats
                        print(f"===== DIRECT GPS MONITOR - {datetime.now().strftime('%H:%M:%S')} =====")
                        print(f"Port: {port} @ {baud_rate} baud")
                        
                        if last_rtk_status:
                            quality = last_rtk_status['quality']
                            quality_color = COLOR_RED
                            if quality == 4:
                                quality_color = COLOR_GREEN
                            elif quality == 5:
                                quality_color = COLOR_YELLOW
                            elif quality == 2:
                                quality_color = COLOR_BLUE
                                
                            print(f"RTK Status: {quality_color}{last_rtk_status['quality_text']}{COLOR_RESET} with {last_rtk_status['satellites']} satellites")
                        
                        print("\nSentence counts:")
                        for stype, count in sorted(sentence_counts.items()):
                            print(f"  {stype}: {count}")
                        print("\nContinuing to monitor... (Screen will clear in 30 seconds)")
                        print("=" * 60)
                        
                    # Small delay to prevent CPU hogging
                    time.sleep(0.01)
                    
                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                    
    except serial.SerialException as e:
        print(f"Error: {e}")
        return False
        
    return True

if __name__ == "__main__":
    # Find available ports
    ports = find_serial_ports()
    
    if not ports:
        print("No serial ports found")
        sys.exit(1)
    
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port}")
    
    # Select port
    choice = input("\nSelect port number (or press Enter for port 1): ")
    port_idx = 0  # Default to first port
    
    if choice.strip():
        try:
            port_idx = int(choice) - 1
            if port_idx < 0 or port_idx >= len(ports):
                print(f"Invalid choice. Using port 1")
                port_idx = 0
        except ValueError:
            print("Invalid input. Using port 1")
    
    port = ports[port_idx]
    
    # Try common baud rates - u-blox typically uses 9600 or 115200
    baud_rates = [115200, 9600, 38400, 4800]
    
    for baud in baud_rates:
        print(f"\nTrying {port} at {baud} baud...")
        if monitor_serial_port(port, baud):
            break
