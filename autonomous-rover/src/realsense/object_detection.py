import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to color frames
align = rs.align(rs.stream.color)

# Initialize object detector (using HOG for people detection)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

try:
    while True:
        # Wait for a coherent pair of frames
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("No frames received")
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            # Object detection
            boxes, weights = hog.detectMultiScale(color_image, winStride=(8, 8))
            
            # Draw boxes and show distances for detected objects
            for i, (x, y, w, h) in enumerate(boxes):
                # Draw rectangle around the detected object
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                # Calculate center point of the box
                center_x = x + w // 2
                center_y = y + h // 2
                
                # Get distance at the center point
                distance = depth_frame.get_distance(center_x, center_y)
                
                # Add text with distance
                cv2.putText(color_image, f"{distance:.2f}m", (x, y - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            # Show images
            cv2.namedWindow('RealSense Object Detection', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Object Detection', images)
            
            if cv2.waitKey(1) == 27:  # ESC key
                break
                
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)
            
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()