import pyrealsense2 as rs
import time
from src.camera import RealSenseCamera
from src.visualization.text_renderer import TextRenderer

def main():
    camera = RealSenseCamera()
    text_renderer = TextRenderer()

    try:
        camera.start_streaming()

        while True:
            depth_frame = camera.get_depth_frame()
            if depth_frame is not None:
                distances = []
                for y in range(480):  # Assuming a resolution of 640x480
                    for x in range(640):
                        distance = depth_frame.get_distance(x, y)
                        distances.append(distance)

                # Visualize distances using text renderer
                text_renderer.render(distances)

            time.sleep(0.1)  # Adjust the sleep time as necessary

    except KeyboardInterrupt:
        print("Measurement stopped by user.")
    finally:
        camera.stop_streaming()

if __name__ == "__main__":
    main()