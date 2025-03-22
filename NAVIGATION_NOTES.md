# Navigation System Notes

## Current Working Configuration (v1.0-tanh-nav)

### Key Components:
- Stanley controller with Tanh-based steering
- Corrected bearing calculation (now matches GPS test and map visualizer)
- Maximum speed: 0.3
- Steering sensitivity: 0.5

### Fixed Issues:
- Parameter ordering in stanley_control was causing incorrect bearing calculations
- Coordinate handling now properly passes (lon, lat) to controllers
- Differential steering now correctly applies speed factors to wheels

### Navigation Files:
- Waypoints are stored in `autonomous-rover/data/polygon_data.json`
- Map visualization available through `map_visualizer.py`

### Operation Notes:
- May see remaining issues with moving base GPS and relative positioning
- For best results, create waypoints in absolute positioning mode

## Testing New Features:
- Run with `--gps-test` to verify bearing calculations
- Check steering with `--real` mode for hardware operation
- Use `serial_gps_monitor.py` for manual control and waypoint creation
