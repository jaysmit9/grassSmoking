#!/usr/bin/env python3

import sys
import os

print(f"Python version: {sys.version}")
print(f"Python executable: {sys.executable}")
print(f"Current working directory: {os.getcwd()}")
print("\nPython path:")
for path in sys.path:
    print(f"  - {path}")

try:
    # Try to import from the local hardware module
    print("\nTrying to import hardware.motor_controller...")
    from hardware.motor_controller import get_motor_controller
    print("Import successful!")
except ImportError as e:
    print(f"Import error: {e}")
    print("\nListing files in current directory:")
    for item in os.listdir('.'):
        print(f"  - {item}")
    
    # Check if hardware directory exists
    if os.path.exists('hardware'):
        print("\nListing files in hardware directory:")
        for item in os.listdir('hardware'):
            print(f"  - {item}")