#!/usr/bin/env python3
# Save as lora_simple_test.py

import serial
import time
import sys
import threading

# Open the serial port
port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
ser = serial.Serial(port, 57600, timeout=0.1)

# Function to continuously read from serial
def reader():
    while True:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            print(f"\nRX: {data.decode('utf-8', errors='replace').strip()}")
            print("> ", end='', flush=True)
        time.sleep(0.1)

# Start reader thread
t = threading.Thread(target=reader, daemon=True)
t.start()

# Main loop for sending
print(f"Simple LoRa test on {port}. Type messages and press Enter.")
print("Press Ctrl+C to exit")
try:
    while True:
        msg = input("> ")
        if msg:
            ser.write((msg + "\n").encode('utf-8'))
            ser.flush()
            print(f"TX: {msg}")
except KeyboardInterrupt:
    ser.close()
    print("\nExiting...")
