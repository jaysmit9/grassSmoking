#!/usr/bin/env python3
import sys
import os
import time
import cv2
import numpy as np
import pyrealsense2 as rs
import logging

# Add the parent directory to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.camera import RealSenseCamera
from src.visualization.text_renderer import TextRenderer
from PyQt5 import QtWidgets, QtCore, QtGui

# Set up logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("InteractiveViewer")

class InteractiveViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Initialize RealSense camera
        try:
            logger.info("Initializing RealSense camera...")
            self.camera = RealSenseCamera(enable_color=True)
        except Exception as e:
            logger.error(f"Failed to initialize camera: {e}")
            QtWidgets.QMessageBox.critical(
                self, 
                "Camera Error", 
                f"Failed to initialize RealSense camera:\n{str(e)}\n\nPlease make sure the camera is connected."
            )
            raise
        
        # Initialize text renderer
        self.text_renderer = TextRenderer()
        
        # Set up the UI
        self.setup_ui()
        
        # Start the update timer
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(33)  # ~30 fps
        
        # Track mouse position for distance measurement
        self.mouse_x = 0
        self.mouse_y = 0
        
        # Store the last depth frame
        self.last_depth_frame = None

        # Counter for reconnection attempts
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.last_frame_time = time.time()
        
    def setup_ui(self):
        # Set window properties
        self.setWindowTitle("RealSense Interactive Viewer")
        self.setGeometry(100, 100, 1200, 700)
        
        # Create main widget and layout
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        
        # Create main layout
        self.main_layout = QtWidgets.QHBoxLayout()
        self.central_widget.setLayout(self.main_layout)
        
        # Left side - depth visualization
        self.left_panel = QtWidgets.QVBoxLayout()
        
        # Depth view
        self.depth_view = QtWidgets.QLabel("Waiting for depth data...")
        self.depth_view.setAlignment(QtCore.Qt.AlignCenter)
        self.depth_view.setMinimumSize(640, 480)
        self.depth_view.mouseMoveEvent = self.mouse_move_event
        self.depth_view.mousePressEvent = self.mouse_press_event
        self.left_panel.addWidget(self.depth_view)
        
        # Distance information
        self.distance_info = QtWidgets.QLabel("Distance: N/A")
        self.distance_info.setAlignment(QtCore.Qt.AlignCenter)
        self.left_panel.addWidget(self.distance_info)
        
        # Colormap selection
        colormap_layout = QtWidgets.QHBoxLayout()
        colormap_layout.addWidget(QtWidgets.QLabel("Colormap:"))
        self.colormap_combo = QtWidgets.QComboBox()
        self.colormap_combo.addItems(['jet', 'rainbow', 'bone', 'hot', 'plasma'])
        self.colormap_combo.currentTextChanged.connect(self.change_colormap)
        colormap_layout.addWidget(self.colormap_combo)
        self.left_panel.addLayout(colormap_layout)
        
        # Add left panel to main layout
        self.main_layout.addLayout(self.left_panel)
        
        # Right side - text visualization & controls
        self.right_panel = QtWidgets.QVBoxLayout()
        
        # Text visualization
        self.text_view = QtWidgets.QTextEdit()
        self.text_view.setReadOnly(True)
        self.text_view.setFontFamily("Courier New")
        self.text_view.setMinimumWidth(400)
        self.right_panel.addWidget(self.text_view)
        
        # Distance summary
        self.distance_summary = QtWidgets.QLabel("Distance Summary: N/A")
        self.right_panel.addWidget(self.distance_summary)
        
        # Add right panel to main layout
        self.main_layout.addLayout(self.right_panel)
        
        # Status bar
        self.statusBar().showMessage("Ready")
        
    def update_frame(self):
        # Check if we need to attempt reconnection
        current_time = time.time()
        if current_time - self.last_frame_time > 5:  # 5 seconds without frames
            if self.reconnect_attempts < self.max_reconnect_attempts:
                self.reconnect_attempts += 1
                self.statusBar().showMessage(f"No frames for 5 seconds. Reconnection attempt {self.reconnect_attempts}/{self.max_reconnect_attempts}...")
                try:
                    self.camera.stop()
                    time.sleep(1)
                    self.camera = RealSenseCamera(enable_color=True)
                    self.last_frame_time = current_time
                except Exception as e:
                    logger.error(f"Reconnection attempt failed: {e}")
            else:
                self.statusBar().showMessage("Failed to reconnect to camera after multiple attempts")
                return
                
        # Get depth frame
        depth_frame = self.camera.get_depth_frame()
        if depth_frame:
            # Update last frame time
            self.last_frame_time = current_time
            self.reconnect_attempts = 0
            
            # Store the frame for mouse events
            self.last_depth_frame = depth_frame
            
            # Update depth visualization
            depth_colormap = self.camera.get_colorized_depth_image()
            if depth_colormap is not None:
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
                self.depth_view.setPixmap(QtGui.QPixmap.fromImage(q_img))
            
            # Update text visualization
            text_viz = self.text_renderer.render(depth_frame, width=80, height=20)
            self.text_view.setText(text_viz)
            
            # Update distance summary
            summary = self.text_renderer.get_distance_summary(depth_frame)
            self.distance_summary.setText(f"Distance Summary: {summary}")
            
            # Check if mouse is over image and update distance
            if 0 <= self.mouse_x < 640 and 0 <= self.mouse_y < 480:
                distance = depth_frame.get_distance(self.mouse_x, self.mouse_y)
                self.distance_info.setText(f"Distance at ({self.mouse_x}, {self.mouse_y}): {distance:.2f} meters")
        else:
            self.statusBar().showMessage("No depth frame available")
            
    def mouse_move_event(self, event):
        # Get the size of the label and the original image
        label_size = self.depth_view.size()
        
        # Calculate the scale factors
        scale_x = 640 / label_size.width()
        scale_y = 480 / label_size.height()
        
        # Calculate the original image coordinates
        self.mouse_x = int(event.x() * scale_x)
        self.mouse_y = int(event.y() * scale_y)
        
    def mouse_press_event(self, event):
        if self.last_depth_frame and 0 <= self.mouse_x < 640 and 0 <= self.mouse_y < 480:
            distance = self.last_depth_frame.get_distance(self.mouse_x, self.mouse_y)
            self.statusBar().showMessage(f"Clicked at ({self.mouse_x}, {self.mouse_y}): {distance:.2f} meters")
    
    def change_colormap(self, colormap_name):
        self.camera.set_colormap(colormap_name)
        
    def closeEvent(self, event):
        # Clean up when closing the application
        self.timer.stop()
        self.camera.stop()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Show a splash screen with instructions
    splash = QtWidgets.QSplashScreen(QtGui.QPixmap())
    splash.showMessage("Initializing RealSense camera...\nPlease wait...", 
                      QtCore.Qt.AlignCenter | QtCore.Qt.AlignBottom, QtCore.Qt.white)
    splash.show()
    app.processEvents()
    
    try:
        viewer = InteractiveViewer()
        splash.finish(viewer)
        viewer.show()
        return app.exec_()
    except Exception as e:
        logger.error(f"Failed to start application: {e}")
        splash.close()
        QtWidgets.QMessageBox.critical(
            None, 
            "Application Error", 
            f"Failed to start the application:\n{str(e)}"
        )
        return 1

if __name__ == "__main__":
    sys.exit(main())