# filepath: /realsense-wrapper/realsense-wrapper/src/config/camera_settings.py

# Configuration settings for the RealSense camera
CAMERA_SETTINGS = {
    'resolution': (640, 480),  # Width, Height
    'frame_rate': 30,          # Frames per second
    'depth_format': 'z16',     # Depth format
    'color_format': 'bgr8',
    'depth_units': 0.001,      # 1mm
    'streaming': True          # Streaming status
}