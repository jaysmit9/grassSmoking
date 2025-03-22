# RealSense Object Detection

A real-time object detection system using Intel RealSense depth cameras with YOLOv3/YOLOv3-tiny.

## Features

- Real-time object detection with distance measurement
- Color-coded bounding boxes based on distance
- Dashboard displaying:
  - FPS counter
  - Detected object counts
  - Closest object information
  - Configurable depth range
- Interactive depth range adjustment
- Automatic timeout for stale detections
- Robust error handling and camera recovery

## Requirements

- Intel RealSense camera (tested with D415/D435)
- Python 3.6+
- OpenCV with DNN support
- Intel RealSense SDK 2.0
- YOLOv3 or YOLOv3-tiny weights and configuration files

## Installation

1. Install the RealSense SDK:

