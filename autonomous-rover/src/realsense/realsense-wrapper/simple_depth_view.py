import sys
import pyrealsense2 as rs
from src.camera import RealSenseCamera
from src.visualization.text_renderer import TextRenderer

def main():
    # Initialize the RealSense camera
    camera = RealSenseCamera()
    
    # Start streaming
    camera.start_streaming()

    # Initialize the text renderer
    text_renderer = TextRenderer()

    try:
        while True:
            # Retrieve depth data
            depth_frame = camera.get_depth_frame()
            if depth_frame is not None:
                # Render the depth data as text
                text_renderer.render(depth_frame)
    except KeyboardInterrupt:
        print("Stopping the depth view.")
    finally:
        camera.stop_streaming()
        sys.exit(0)

if __name__ == "__main__":
    main()