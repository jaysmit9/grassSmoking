import pyrealsense2 as rs
import numpy as np
import cv2
import time
import logging
from src.config.camera_settings import CAMERA_SETTINGS
from src.utils.distance_calculations import convert_raw_depth_to_meters

# Set up logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("RealSenseCamera")

class RealSenseCamera:
    def __init__(self, enable_color=False, timeout=5000):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.timeout = timeout
        self.connected = False
        
        try:
            # Try to find connected devices
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(list(devices)) == 0:
                logger.error("No RealSense devices connected!")
                raise RuntimeError("No RealSense devices connected!")
            
            logger.info(f"Found {len(list(devices))} RealSense devices")
            
            # Configure streams based on settings
            self.config.enable_stream(
                rs.stream.depth, 
                CAMERA_SETTINGS['resolution'][0], 
                CAMERA_SETTINGS['resolution'][1], 
                rs.format.z16, 
                CAMERA_SETTINGS['frame_rate']
            )
            
            if enable_color:
                self.config.enable_stream(
                    rs.stream.color, 
                    CAMERA_SETTINGS['resolution'][0], 
                    CAMERA_SETTINGS['resolution'][1], 
                    rs.format.bgr8, 
                    CAMERA_SETTINGS['frame_rate']
                )
            
            # Create align object if color is enabled
            self.align = None
            if enable_color:
                self.align = rs.align(rs.stream.color)
            
            # Start streaming
            logger.info("Starting RealSense pipeline...")
            self.profile = self.pipeline.start(self.config)
            
            # Get depth scale for distance calculations
            self.depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = self.depth_sensor.get_depth_scale()
            
            # Default colormap for visualization
            self.colormap = cv2.COLORMAP_JET
            
            # Try to get a frame to confirm everything is working
            logger.info("Waiting for first frame...")
            self.get_depth_frame(first_frame=True)
            self.connected = True
            logger.info("RealSense camera initialized successfully")
            
        except Exception as e:
            logger.error(f"Failed to initialize RealSense camera: {e}")
            self.stop()
            raise

    def get_depth_frame(self, first_frame=False):
        """
        Get the latest depth frame from the camera.
        
        Args:
            first_frame: Whether this is the first frame request (used for initial setup)
        
        Returns:
            Depth frame or None if timeout
        """
        if not self.connected and not first_frame:
            logger.error("Camera is not connected")
            return None
            
        try:
            # Try to wait for frames with the specified timeout
            frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                logger.warning("Got frames, but no depth frame available")
            return depth_frame
        except RuntimeError as e:
            if "Frame didn't arrive within" in str(e):
                logger.warning(f"Frame timeout after {self.timeout}ms. Check camera connection.")
                if first_frame:
                    raise  # Raise on first frame to fail initialization
            else:
                logger.error(f"Error getting depth frame: {e}")
            return None
    
    def get_color_frame(self):
        """Get the latest color frame from the camera."""
        if not self.connected:
            logger.error("Camera is not connected")
            return None
            
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout)
            color_frame = frames.get_color_frame()
            if not color_frame:
                logger.warning("Got frames, but no color frame available")
            return color_frame
        except RuntimeError as e:
            logger.error(f"Error getting color frame: {e}")
            return None
    
    def get_aligned_frames(self):
        """Get aligned depth and color frames."""
        if not self.connected:
            logger.error("Camera is not connected")
            return None, None
            
        if not self.align:
            logger.error("Camera was not initialized with color stream")
            return None, None
        
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=self.timeout)
            aligned_frames = self.align.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            return depth_frame, color_frame
        except RuntimeError as e:
            logger.error(f"Error getting aligned frames: {e}")
            return None, None
    
    def get_distance(self, x, y):
        """Get distance at a specific pixel."""
        depth_frame = self.get_depth_frame()
        if depth_frame:
            dist = depth_frame.get_distance(x, y)
            return convert_raw_depth_to_meters(dist)
        return None
    
    def get_depth_image(self):
        """Convert depth frame to numpy array."""
        depth_frame = self.get_depth_frame()
        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            return depth_image
        return None
    
    def get_colorized_depth_image(self):
        """Get depth frame as colorized image."""
        depth_image = self.get_depth_image()
        if depth_image is not None:
            # Apply color map to depth image (adjust alpha for visualization)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                self.colormap
            )
            return depth_colormap
        return None
    
    def set_colormap(self, colormap_type):
        """Set the colormap for depth visualization."""
        colormaps = {
            'jet': cv2.COLORMAP_JET,
            'rainbow': cv2.COLORMAP_RAINBOW,
            'bone': cv2.COLORMAP_BONE,
            'hot': cv2.COLORMAP_HOT,
            'plasma': cv2.COLORMAP_PLASMA
        }
        
        if colormap_type in colormaps:
            self.colormap = colormaps[colormap_type]
            return True
        return False
    
    def stop(self):
        """Stop the pipeline."""
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
            self.connected = False
            logger.info("RealSense pipeline stopped")
        except Exception as e:
            logger.error(f"Error stopping pipeline: {e}")