import pyrealsense2 as rs
import numpy as np
import cv2
import time
import os
import sys
import traceback

# Check if the required files exist
required_files = ["yolov3.weights", "yolov3.cfg", "coco.names"]
for file in required_files:
    if not os.path.isfile(file):
        print(f"Error: {file} not found in the current directory")
        sys.exit(1)

print("Initializing camera...")
# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
try:
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Check available devices
    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("No RealSense devices detected!")
        sys.exit(1)
    else:
        print(f"Found {len(devices)} RealSense device(s)")
        for dev in devices:
            print(f" - {dev.get_info(rs.camera_info.name)}")
    
    # Start streaming
    print("Starting camera pipeline...")
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    print("Camera pipeline started successfully")
    
    # Load YOLO
    print("Loading YOLO model...")
    net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
    layer_names = net.getLayerNames()
    try:
        unconnected_layers = net.getUnconnectedOutLayers()
        # Handle both OpenCV 4.x and 3.x versions
        if isinstance(unconnected_layers[0], (list, tuple)):
            output_layers = [layer_names[i[0] - 1] for i in unconnected_layers]
        else:
            output_layers = [layer_names[i - 1] for i in unconnected_layers]
    except Exception as e:
        print(f"Error getting output layers: {e}")
        output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]
    
    print("Loading class names...")
    classes = []
    with open("coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    print(f"Loaded {len(classes)} classes")
    
    print("Starting detection loop...")
    try:
        while True:
            try:
                print("Waiting for frames...")
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                print("Got frames")
                aligned_frames = align.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    print("Missing frames, continuing...")
                    continue
                
                # Convert images to numpy arrays
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                print("Running object detection...")
                # YOLO object detection
                height, width, channels = color_image.shape
                blob = cv2.dnn.blobFromImage(color_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
                net.setInput(blob)
                outs = net.forward(output_layers)
                
                # Showing information on the screen
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
                
                print(f"Found {len(boxes)} potential objects")
                
                # Apply non-max suppression
                if boxes:
                    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
                    print(f"After NMS: {len(indexes)} objects")
                    
                    for i in range(len(boxes)):
                        if i in indexes:
                            x, y, w, h = boxes[i]
                            label = str(classes[class_ids[i]])
                            confidence = confidences[i]
                            
                            # Get distance
                            dist = depth_frame.get_distance(x + w//2, y + h//2)
                            
                            # Draw bounding box
                            cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                            cv2.putText(color_image, f"{label} {confidence:.2f} {dist:.2f}m", 
                                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            print(f"Detected: {label} ({confidence:.2f}) at {dist:.2f}m")
                
                # Apply colormap on depth image
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                
                # Stack images horizontally
                images = np.hstack((color_image, depth_colormap))
                
                # Show images
                print("Displaying images")
                cv2.namedWindow('RealSense Deep Learning Detection', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense Deep Learning Detection', images)
                
                key = cv2.waitKey(1)
                if key == 27:  # ESC key
                    print("ESC pressed, exiting...")
                    break
                    
            except Exception as e:
                print(f"Error in detection loop: {e}")
                print(traceback.format_exc())
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("Keyboard interrupt received, stopping...")
    
except Exception as e:
    print(f"Setup error: {e}")
    print(traceback.format_exc())
    
finally:
    # Stop streaming
    print("Stopping pipeline...")
    pipeline.stop()
    cv2.destroyAllWindows()
    print("Pipeline stopped, windows closed")