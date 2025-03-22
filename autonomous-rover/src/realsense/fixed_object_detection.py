import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
import traceback

# Check if the required files exist
required_files = ["yolov3-tiny.weights", "yolov3-tiny.cfg", "coco.names"]
for file in required_files:
    if not os.path.isfile(file):
        print(f"Error: {file} not found in the current directory")
        sys.exit(1)

print("Initializing camera...")
# Configure streams
pipeline = rs.pipeline()
config = rs.config()

try:
    # First add depth stream (this works in simple_depth)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # Then try adding color stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Start streaming
    print("Starting camera pipeline...")
    profile = pipeline.start(config)
    print("Camera pipeline started successfully")
    
    # Get the depth sensor's depth scale
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale is: {depth_scale}")
    
    # Create align object - but only use it if we successfully get frames
    align = rs.align(rs.stream.color)
    
    # Load YOLO
    print("Loading YOLO model...")
    net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
    layer_names = net.getLayerNames()
    
    # Handle output layers in a simpler way
    output_layers = []
    for i in net.getUnconnectedOutLayers():
        if isinstance(i, (list, tuple)):
            output_layers.append(layer_names[i[0] - 1])
        else:
            output_layers.append(layer_names[i - 1])
    
    print("Loading class names...")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    print(f"Loaded {len(classes)} classes")
    
    print("Starting detection loop...")
    try:
        while True:
            # Wait for frames - USE SIMPLE APPROACH LIKE IN SIMPLE_DEPTH
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if not frames:
                print("No frames received")
                continue
            
            # Try to align frames safely
            try:
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
            except Exception as e:
                print(f"Error during frame alignment: {e}")
                # Fall back to unaligned frames
                print("Using unaligned frames instead")
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                print("Missing frames, continuing...")
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # YOLO object detection
            height, width, channels = color_image.shape
            blob = cv2.dnn.blobFromImage(color_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)
            
            # Process detections
            class_ids = []
            confidences = []
            boxes = []
            
            for out in outs:
                for detection in out:
                    scores = detection[5:]
                    class_id = np.argmax(scores)
                    confidence = scores[class_id]
                    if confidence > 0.5:
                        # Object detected
                        center_x = int(detection[0] * width)
                        center_y = int(detection[1] * height)
                        w = int(detection[2] * width)
                        h = int(detection[3] * height)
                        
                        # Rectangle coordinates
                        x = int(center_x - w / 2)
                        y = int(center_y - h / 2)
                        
                        boxes.append([x, y, w, h])
                        confidences.append(float(confidence))
                        class_ids.append(class_id)
            
            # Apply non-max suppression
            if boxes:
                try:
                    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
                    # Handle different return types depending on OpenCV version
                    indexes = indexes.flatten() if isinstance(indexes, np.ndarray) else indexes
                    
                    for i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(classes[class_ids[i]])
                        confidence = confidences[i]
                        
                        # Make sure coordinates are within image bounds
                        center_x = min(max(0, x + w//2), width-1)
                        center_y = min(max(0, y + h//2), height-1)
                        
                        # Get distance - handle potential errors
                        try:
                            dist = depth_frame.get_distance(center_x, center_y)
                        except:
                            dist = 0
                            print("Could not get distance")
                        
                        # Draw bounding box
                        cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(color_image, f"{label} {confidence:.2f} {dist:.2f}m", 
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        print(f"Detected: {label} ({confidence:.2f}) at {dist:.2f}m")
                except Exception as e:
                    print(f"Error in NMS or drawing: {e}")
            
            # Apply colormap on depth image
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            # Stack images horizontally - handle different sizes
            try:
                if depth_colormap.shape[0] == color_image.shape[0]:
                    images = np.hstack((color_image, depth_colormap))
                else:
                    # Resize to match height
                    h, w, _ = color_image.shape
                    depth_colormap = cv2.resize(depth_colormap, (int(w * (depth_colormap.shape[0] / color_image.shape[0])), h))
                    images = np.hstack((color_image, depth_colormap))
            except Exception as e:
                print(f"Error stacking images: {e}")
                images = color_image  # Fallback to just showing color image
            
            # Show images
            cv2.namedWindow('RealSense Object Detection', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense Object Detection', images)
            
            key = cv2.waitKey(1)
            if key == 27:  # ESC key
                print("ESC pressed, exiting...")
                break
                
    except KeyboardInterrupt:
        print("Keyboard interrupt, stopping...")
    
except Exception as e:
    print(f"Setup error: {e}")
    traceback.print_exc()
    
finally:
    # Stop streaming
    print("Stopping pipeline...")
    try:
        pipeline.stop()
    except Exception as e:
        print(f"Error stopping pipeline: {e}")
        
    cv2.destroyAllWindows()
    print("Pipeline stopped, windows closed")
