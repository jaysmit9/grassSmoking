import pyrealsense2 as rs
import numpy as np
import cv2
import time
import sys
import os

def get_device_info():
    """Get information about connected RealSense devices"""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense devices connected!")
        return None
    
    for i, dev in enumerate(devices):
        print(f"Device {i}: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial Number: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware Version: {dev.get_info(rs.camera_info.firmware_version)}")
        return dev  # Return first device
    
    return None

def log_system_info():
    """Log system information that might be relevant"""
    print("\n=== System Information ===")
    print(f"PyRealSense2 version: {rs.__version__}")
    print(f"OpenCV version: {cv2.__version__}")
    print(f"NumPy version: {np.__version__}")
    print(f"Python version: {sys.version}")
    
    try:
        import psutil
        print(f"CPU usage: {psutil.cpu_percent(interval=1)}%")
        print(f"Memory usage: {psutil.virtual_memory().percent}%")
        print(f"Available memory: {psutil.virtual_memory().available / (1024 * 1024):.2f} MB")
    except ImportError:
        print("psutil not available - can't show system resource usage")
    
    # Log USB information on Linux
    if os.name == 'posix':
        try:
            import subprocess
            usb_info = subprocess.check_output(['lsusb']).decode('utf-8')
            print("\nUSB Devices:")
            for line in usb_info.split('\n'):
                if 'Intel' in line:  # Looking for RealSense devices
                    print(f"  {line}")
        except:
            print("Could not retrieve USB information")

def main():
    print("\n=== RealSense Simple Depth Demo (Enhanced) ===")
    
    # Log system and device info
    log_system_info()
    device = get_device_info()
    if not device:
        print("No RealSense device found. Exiting.")
        return
    
    # Configure depth stream
    pipe = rs.pipeline()
    config = rs.config()
    
    # Try with more stable framerates - 30 fps is the standard
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
    
    # Get device product line for setting a supporting resolution
    device_product_line = str(device.get_info(rs.camera_info.product_line))
    print(f"\nUsing device product line: {device_product_line}")
    
    # Additional configuration options for stability
    cfg = pipe.start(config)
    dev = cfg.get_device()
    depth_sensor = dev.first_depth_sensor()
    
    # Print supported options
    print("\nSupported options:")
    for option in depth_sensor.get_supported_options():
        try:
            option_value = depth_sensor.get_option(option)
            option_range = depth_sensor.get_option_range(option)
            option_desc = depth_sensor.get_option_description(option)
            print(f"  {option}: {option_value} (range: {option_range.min}-{option_range.max}, default: {option_range.default})")
            print(f"    Description: {option_desc}")
        except:
            pass
    
    # Try to optimize for stability
    try:
        # Set auto exposure mode to favor better stability
        if depth_sensor.supports(rs.option.enable_auto_exposure):
            depth_sensor.set_option(rs.option.enable_auto_exposure, 1)
        
        # Set laser power to max for better depth data
        if depth_sensor.supports(rs.option.laser_power):
            laser_range = depth_sensor.get_option_range(rs.option.laser_power)
            depth_sensor.set_option(rs.option.laser_power, laser_range.max * 0.8)  # 80% of max power
        
        # Disable motion correction if available
        if depth_sensor.supports(rs.option.motion_range):
            depth_sensor.set_option(rs.option.motion_range, 0)
    except Exception as e:
        print(f"Error configuring sensor options: {e}")
    
    # Performance tracking
    frame_count = 0
    error_count = 0
    start_time = time.time()
    last_fps_update = start_time
    
    # Wait for camera to stabilize
    print("\nWaiting for camera to stabilize...")
    time.sleep(2)
    print("Starting main loop")
    
    try:
        while True:
            loop_start = time.time()
            
            # Wait for a coherent pair of frames with longer timeout
            try:
                frames = pipe.wait_for_frames(timeout_ms=2000)  # Increase timeout
                depth_frame = frames.get_depth_frame()
                
                if not depth_frame:
                    print("No depth frame received")
                    error_count += 1
                    time.sleep(0.1)
                    continue
                
                # Update frame counter
                frame_count += 1
                
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                
                # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                # Show images
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', depth_colormap)
                
                # Show distance at center of image
                center_x = depth_image.shape[1] // 2
                center_y = depth_image.shape[0] // 2
                distance = depth_frame.get_distance(center_x, center_y)
                
                # Calculate FPS
                current_time = time.time()
                elapsed = current_time - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                # Only update FPS display every second
                if current_time - last_fps_update >= 1.0:
                    print(f"FPS: {fps:.1f}, Errors: {error_count}, Distance at center: {distance:.2f}m")
                    last_fps_update = current_time
                
                # Display stats on image
                stats_text = f"FPS: {fps:.1f} | Errors: {error_count} | Dist: {distance:.2f}m"
                cv2.putText(depth_colormap, stats_text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow('RealSense', depth_colormap)
                
                # Process time measurement for loop pacing
                process_time = time.time() - loop_start
                
                # If processing is too fast, add a slight delay 
                # This can sometimes help with USB stability
                if process_time < 0.01:  # Less than 10ms processing time
                    time.sleep(0.01)  # Add a small delay
                
                if cv2.waitKey(1) == 27:  # ESC key
                    break
                    
            except Exception as e:
                error_count += 1
                print(f"Error: {e}")
                time.sleep(0.2)  # Give system time to recover
                
    finally:
        # Stop streaming and print stats
        pipe.stop()
        cv2.destroyAllWindows()
        
        elapsed = time.time() - start_time
        print(f"\nRun summary:")
        print(f"  Duration: {elapsed:.1f} seconds")
        print(f"  Frames captured: {frame_count}")
        print(f"  Errors: {error_count}")
        print(f"  Average FPS: {frame_count / elapsed if elapsed > 0 else 0:.2f}")
        print(f"  Error rate: {error_count / (frame_count + error_count) * 100 if (frame_count + error_count) > 0 else 0:.2f}%")

if __name__ == "__main__":
    main()