# RealSense Wrapper

This project provides a Python wrapper for Intel RealSense cameras, enabling users to visualize distances in the camera's field of view. It includes functionalities for both text-based and graphical visualizations of depth data.

## Features

- Connect to Intel RealSense cameras and stream depth data.
- Text-based visualization of distance data.
- Graphical visualization using libraries like OpenCV or Matplotlib.
- Utility functions for distance calculations and depth data processing.

## Installation

To install the required dependencies, run:

```
pip install -r requirements.txt
```

## Usage

### Simple Depth View

To visualize depth data in a simple text format, run the following example:

```
python examples/simple_depth_view.py
```

### Distance Measurement

To measure distances using the RealSense camera, execute:

```
python examples/distance_measurement.py
```

## Configuration

Camera settings such as resolution and frame rate can be adjusted in the `src/config/camera_settings.py` file.

## Testing

To run the unit tests for the project, use:

```
pytest
```

## Contributing

Contributions are welcome! Please feel free to submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the Apache 2.0 License. See the LICENSE file for more details.