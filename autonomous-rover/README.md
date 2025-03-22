# Autonomous Rover Project

## Overview
The Autonomous Rover project is designed to navigate a rover using GPS waypoints. It integrates various components including a Stanley controller for navigation, GPS monitoring for position tracking, and motor control for movement. The project supports both simulation and real-world operation.

## Project Structure
```
autonomous-rover
├── src
│   ├── controllers
│   │   └── stanley_controller.py
│   ├── hardware
│   │   ├── gps_monitor.py
│   │   └── motor_controller.py
│   ├── utils
│   │   ├── coordinate_utils.py
│   │   └── path_planning.py
│   ├── simulation
│   │   ├── sim_gps.py
│   │   └── sim_motors.py
│   └── main.py
├── config
│   ├── controller_params.json
│   └── rover_config.json
├── data
│   └── polygon_data.json
├── tests
│   ├── test_controller.py
│   └── test_gps.py
├── requirements.txt
└── README.md
```

## Installation
1. Clone the repository:
   ```
   git clone <repository-url>
   cd autonomous-rover
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage
To run the rover in simulation mode, use the following command:
```
python src/main.py --sim
```

To operate the rover in real mode, use:
```
python src/main.py --real
```

## Components
- **Stanley Controller**: Implements the navigation logic based on GPS waypoints.
- **GPS Monitor**: Integrates with GPS hardware to retrieve and process location data.
- **Motor Controller**: Manages the rover's motors for movement based on commands from the controller.
- **Utilities**: Provides functions for coordinate calculations and path planning.
- **Simulation**: Includes mock implementations for GPS and motor controls for testing purposes.

## Testing
Unit tests are provided to ensure the functionality of the controller and GPS monitoring. To run the tests, use:
```
pytest
```

## Contributing
Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License
This project is licensed under the MIT License. See the LICENSE file for details.