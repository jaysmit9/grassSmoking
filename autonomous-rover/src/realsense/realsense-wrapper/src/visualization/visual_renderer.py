import numpy as np
import cv2
from PyQt5 import QtWidgets, QtCore, QtGui
from src.camera import RealSenseCamera

class VisualRenderer:
    def __init__(self, camera=None):
        self.camera = camera or RealSenseCamera(enable_color=True)
        
        # Initialize Qt application
        self.app = QtWidgets.QApplication([])
        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle("RealSense Depth Visualization")
        
        # Create central widget and layout
        self.central_widget = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout()
        
        # Create image label for displaying depth data
        self.image_label = QtWidgets.QLabel()
        self.layout.addWidget(self.image_label)
        
        # Create info label for displaying distance data
        self.info_label = QtWidgets.QLabel("Click on the image to measure distance")
        self.info_label.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.info_label)
        
        # Set up the central widget
        self.central_widget.setLayout(self.layout)
        self.window.setCentralWidget(self.central_widget)
        
        # Set up the timer for updating frames
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        
        # Enable mouse click event on the image label
        self.image_label.setMouseTracking(True)
        self.image_label.mousePressEvent = self.get_distance_at_point
        
        # Track the last depth image for mouse events
        self.last_depth_frame = None
        self.last_color_image = None
        self.last_depth_colormap = None

    def update_frame(self):
        """Update the frame displayed in the window."""
        depth_frame = self.camera.get_depth_frame()
        if not depth_frame:
            return
            
        self.last_depth_frame = depth_frame
        
        # Get colorized depth image
        depth_colormap = self.camera.get_colorized_depth_image()
        if depth_colormap is None:
            return
            
        self.last_depth_colormap = depth_colormap
        
        # Convert OpenCV image to QImage
        height, width, channel = depth_colormap.shape
        bytes_per_line = 3 * width
        q_img = QtGui.QImage(
            depth_colormap.data,
            width, 
            height, 
            bytes_per_line, 
            QtGui.QImage.Format_RGB888
        ).rgbSwapped()  # Convert BGR to RGB
        
        # Set the QImage to the QLabel
        self.image_label.setPixmap(QtGui.QPixmap.fromImage(q_img))
        self.image_label.setScaledContents(True)
        self.image_label.setMinimumSize(640, 480)

    def get_distance_at_point(self, event):
        """Get distance at the clicked point."""
        if self.last_depth_frame is None:
            return
            
        # Get the size of the label and the original image
        label_size = self.image_label.size()
        image_size = self.last_depth_colormap.shape
        
        # Calculate the scale factors
        scale_x = image_size[1] / label_size.width()
        scale_y = image_size[0] / label_size.height()
        
        # Calculate the original image coordinates
        x = int(event.x() * scale_x)
        y = int(event.y() * scale_y)
        
        # Get distance at this point
        distance = self.last_depth_frame.get_distance(x, y)
        
        # Update info label
        self.info_label.setText(f"Distance at ({x}, {y}): {distance:.2f} meters")

    def run(self):
        """Run the visualization."""
        self.window.show()
        self.timer.start(33)  # ~30 fps
        return self.app.exec_()

    def stop(self):
        """Stop the visualization."""
        self.timer.stop()
        self.camera.stop()