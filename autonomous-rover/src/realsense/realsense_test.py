import pyrealsense2 as rs
import numpy as np
import cv2
import time

print("Starting RealSense test...")

try:
    # Create a context object to enumerate devices
    ctx = rs.context()
    print(f"Found {len(ctx.devices)} device(s):")
    
    for i, dev in enumerate(ctx.devices):
        print(f"  {i}: {dev.get_info(rs.camera_info.name)} " +
              f"(S/N: {dev.get_info(rs.camera_info.serial_number)})")
    
    if len(ctx.devices) == 0:
        print("No RealSense devices found! Check connections.")
        exit(1)
    
    # Choose the first device
    print("Using the first available device")
    dev = ctx.devices[0]
    print(f"Using device: {dev.get_info(rs.camera_info.name)} " +
          f"(S/N: {dev.get_info(rs.camera_info.serial_number)})")
    
    # Create a pipeline specifically for this device
    pipeline = rs.pipeline(ctx)
    config = rs.config()
    
    # Enable device by serial number instead of by index
    serial_number = dev.get_info(rs.camera_info.serial_number)
    config.enable_device(serial_number)
    
    # Configure streams
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    print("Starting pipeline...")
    profile = pipeline.start(config)
    
    # Create align object
    align = rs.align(rs.stream.color)
    
    # Process frames for 30 seconds
    start_time = time.time()
    frame_count = 0
    
    while time.time() - start_time < 30:
        try:
            # Wait for frames
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if not frames:
                print("No frames received in timeout period")
                continue
                
            frame_count += 1
            
            # Align frames
            aligned_frames = align.process(frames)
            
            # Get frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("Missing depth or color frame")
                continue
            
            # Convert to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # Stack images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            
            # Print distance at center
            center_x, center_y = depth_image.shape[1]//2, depth_image.shape[0]//2
            distance = depth_frame.get_distance(center_x, center_y)
            print(f"Frame #{frame_count}: Distance at center: {distance:.2f}m")
            
            if cv2.waitKey(1) == 27:  # ESC key
                break
                
        except Exception as e:
            print(f"Error processing frame: {e}")
    
    # Report statistics
    elapsed = time.time() - start_time
    fps = frame_count / elapsed
    print(f"Captured {frame_count} frames in {elapsed:.1f} seconds ({fps:.2f} FPS)")
    
except Exception as e:
    print(f"Error: {e}")
    
finally:
    try:
        pipeline.stop()
    except:
        pass
    
    cv2.destroyAllWindows()
    print("Test complete")