import pyrealsense2 as rs
import sys

print("Python version:", sys.version)
print("RealSense SDK version:", rs.__version__)
print("Checking for RealSense devices...")

try:
    # Just create a context and list devices - nothing else
    ctx = rs.context()
    print(f"Found {len(ctx.devices)} device(s)")
    
    for i, dev in enumerate(ctx.devices):
        try:
            print(f"Device {i}:")
            print(f"  Name: {dev.get_info(rs.camera_info.name)}")
            print(f"  Serial Number: {dev.get_info(rs.camera_info.serial_number)}")
            print(f"  USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")
            print(f"  Firmware Version: {dev.get_info(rs.camera_info.firmware_version)}")
            print(f"  Physical Port: {dev.get_info(rs.camera_info.physical_port)}")
        except Exception as e:
            print(f"  Error getting device info: {e}")
    
except Exception as e:
    print(f"Error during device enumeration: {e}")

print("Test complete")