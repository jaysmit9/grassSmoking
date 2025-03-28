#!/usr/bin/env python3

import time
import math
import threading
import logging
import socket
import json
from geopy.distance import geodesic
import json

# Configure logging
logger = logging.getLogger("rover.hardware.gpsd")

class GPSMonitor:
    def __init__(self, main_port='/dev/ttyACM0', secondary_port='/dev/ttyACM1', 
                 baud_rate=115200, timeout=0.1, use_dual_gps=True, simulation_mode=False):
        """Initialize GPS monitor"""
        # Add thread lock for all instances
        self.lock = threading.RLock()
        
        self.simulation_mode = simulation_mode
        self.use_dual_gps = use_dual_gps
        self.timeout = timeout
        self.L = 1.0  # Wheelbase length in meters
        
        # For simulation calculations
        self.lat_scale = 1.0 / 111000  # Approximate conversion (meters to degrees)
        self.lon_scale = 1.0 / 111000  # Will be refined based on latitude
        
        # Initialize simulation variables
        if simulation_mode:
            self._init_simulation()
        else:
            # Your existing hardware initialization code
            # For debugging purposes, print object ID to identify different instances
            logger.info(f"Creating GPSMonitor instance {id(self)} with simulation_mode={simulation_mode}")
            
            # GPS data storage (matching serial_gps_monitor.py structure)
            self.gps_data = {
                "front": {
                    "lat": None, "lon": None, "heading": None, 
                    "speed": None, "fix_quality": 0, "last_update": 0,
                    "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
                },
                "rear": {
                    "lat": None, "lon": None, "heading": None, 
                    "speed": None, "fix_quality": 0, "last_update": 0,
                    "messages_per_sec": 0, "message_count": 0, "last_message_time": time.time()
                }
            }
            
            logger.info(f"Initializing GPS Monitor in {'SIMULATION' if simulation_mode else 'HARDWARE'} mode")
            
            if not self.simulation_mode:
                try:
                    import serial
                    self.main_serial = serial.Serial(main_port, baud_rate, timeout=timeout)
                    logger.info(f"Connected to main GPS on {main_port}")
                    
                    if use_dual_gps:
                        self.secondary_serial = serial.Serial(secondary_port, baud_rate, timeout=timeout)
                        logger.info(f"Connected to secondary GPS on {secondary_port}")
                    
                    # Start reader threads
                    self._start_hardware_readers()
                except Exception as e:
                    logger.error(f"Failed to initialize hardware GPS: {e}")
                    logger.info("Falling back to simulation mode")
                    self.simulation_mode = True
            
            if self.simulation_mode:
                self._start_simulation()
    
    def _init_simulation(self):
        """Initialize simulation parameters"""
        logger.info("Initializing GPS simulation")
        
        # Load default waypoints for simulation
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(script_dir))
        waypoints_file = os.path.join(project_root, "data", "polygon_data.json")
        
        try:
            with open(waypoints_file) as f:
                data = json.load(f)
            
            # Extract waypoints
            self.sim_waypoints = np.array([(point['lat'], point['lon']) for point in data])
            logger.info(f"Loaded {len(self.sim_waypoints)} waypoints for GPS simulation")
            
            # Fix: Use actual GPS coordinates for initial position
            self.sim_x = float(self.sim_waypoints[0][0])  # Cast to float to ensure numeric type
            self.sim_y = float(self.sim_waypoints[0][1])
            logger.info(f"Initial position set to: ({self.sim_x:.6f}, {self.sim_y:.6f})")
            
            self.sim_yaw = 0.0  # Initial heading (North)
            self.sim_speed = 0.0  # Initial speed
            
            # FIXED: Use amplified scaling factors for better simulation
            self.lat_scale = 1.0 / 11100  # 10x larger for faster simulation
            self.lon_scale = 1.0 / (11100 * np.cos(np.radians(self.sim_x)))  # 10x larger
            logger.info(f"Using amplified simulation scales: lat_scale={self.lat_scale:.8f}, lon_scale={self.lon_scale:.8f}")
            
        except Exception as e:
            logger.error(f"Failed to load simulation waypoints: {e}", exc_info=True)
            # Use actual GPS coordinates for default values
            self.sim_waypoints = np.array([[34.151977, -77.866983], [34.152077, -77.866783]])
            self.sim_x = 34.151977
            self.sim_y = -77.866983
            logger.info(f"Using default position: ({self.sim_x}, {self.sim_y})")
            self.sim_yaw = 0.0
            self.sim_speed = 0.0
            # FIXED: Use amplified scaling factors here too
            self.lat_scale = 1.0 / 11100  # 10x larger
            self.lon_scale = 1.0 / (11100 * np.cos(np.radians(self.sim_x)))  # 10x larger
    
    def _start_hardware_readers(self):
        """Start threads to read from real GPS hardware"""
        def gps_reader_thread(serial_port, gps_position):
            import serial
            try:
                buffer = ""
                while True:
                    try:
                        # Read available data
                        if hasattr(serial_port, 'in_waiting') and serial_port.in_waiting:
                            data = serial_port.read(serial_port.in_waiting)
                            
                            # Convert bytes to string
                            text = data.decode('ascii', errors='replace')
                            buffer += text
                            
                            # Process any complete sentences in the buffer
                            while '\n' in buffer:
                                idx = buffer.find('\n')
                                sentence = buffer[:idx].strip()
                                buffer = buffer[idx+1:]
                                
                                # Parse the sentence
                                if sentence:
                                    self._parse_nmea(sentence, gps_position)
                    
                    except UnicodeDecodeError:
                        buffer = ""  # Reset on bad data
                    except Exception as e:
                        logger.error(f"Error in {gps_position} GPS reader: {e}")
                        time.sleep(1)  # Avoid rapid error loops
                    
                    # Small delay to prevent CPU hogging
                    time.sleep(0.01)
                    
            except Exception as e:
                logger.error(f"GPS reader thread error: {e}")
class GpsdClient:
    """Client for connecting to and reading from GPSD daemon"""
    
    def __init__(self, host='localhost', port=2947, debug=False):
        """Initialize GPSD client"""
        self.host = host
        self.port = port
        self.socket = None
        self.debug = debug
        self.devices = {}  # Store info about each device
        self.last_data_time = time.time()
        self.connect()
    
    def log(self, message):
        """Log debug messages"""
        if self.debug:
            logger.debug(f"GPSD: {message}")
    
    def connect(self):
        """Connect to the GPSD daemon"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            
            # First check devices
            self.socket.send(b'?DEVICES;\n')
            time.sleep(0.5)
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"DEVICES response: {data}")
            
            try:
                # Parse device list to see what's available
                for line in data.split('\n'):
                    if not line.strip():
                        continue
                    report = json.loads(line)
                    if report.get('class') == 'DEVICES':
                        devices = report.get('devices', [])
                        self.log(f"Found {len(devices)} devices:")
                        for device in devices:
                            self.log(f"  Path: {device.get('path')}, Driver: {device.get('driver')}")
            except json.JSONDecodeError as e:
                self.log(f"JSON parse error for DEVICES: {e}")
            
            # Enable watching with device field reporting
            watch_command = b'?WATCH={"enable":true,"json":true,"device":true,"nmea":true,"raw":1};\n'
            self.socket.send(watch_command)
            self.log(f"Sent WATCH command: {watch_command.decode()}")
            time.sleep(0.5)
            
            data = self.socket.recv(4096).decode('utf-8').strip()
            self.log(f"WATCH response: {data}")
            
            return True
        except Exception as e:
            logger.error(f"Error connecting to GPSD: {e}")
            return False
    
    def process_reports(self, timeout=1.0):
        """Process all available reports from GPSD"""
        if not self.socket:
            return None
            
        try:
            # Use a shorter timeout to process data chunks
            self.socket.settimeout(timeout)
            data = self.socket.recv(8192).decode('utf-8').strip()
            
            if not data:
                return None
                
            self.log(f"Received data length: {len(data)} bytes")
            
            # Process each line as a separate JSON object
            reports = []
            for line in data.split('\n'):
                if not line.strip():
                    continue
                try:
                    report = json.loads(line)
                    reports.append(report)
                    
                    # Track devices based on their path
                    if report.get('class') == 'TPV' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Get extended information
                        status = report.get('status', 0)
                        mode = report.get('mode', 0)
                        
                        # Determine the complete fix mode
                        fix_mode = self.determine_fix_mode(report, mode, status)
                        
                        # Store device info
                        self.devices[device_path] = {
                            'last_update': time.time(),
                            'lat': report.get('lat', 0.0),
                            'lon': report.get('lon', 0.0),
                            'alt': report.get('alt', 0.0),
                            'track': report.get('track', 0.0),  # Heading/Course
                            'speed': report.get('speed', 0.0),  # Speed in m/s
                            'mode': mode,
                            'status': status,
                            'fix_mode': fix_mode,
                            'eph': report.get('eph', 0.0),  # Horizontal position error
                            'epv': report.get('epv', 0.0),  # Vertical position error
                            'fix_quality': self.map_fix_quality(mode, status, fix_mode),
                            'messages_count': 1 + self.devices.get(device_path, {}).get('messages_count', 0)
                        }
                    
                    # Process SKY reports for satellite info
                    elif report.get('class') == 'SKY' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Update satellite info if device exists
                        if device_path in self.devices:
                            self.devices[device_path].update({
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            })
                        else:
                            # Create new device entry if it doesn't exist
                            self.devices[device_path] = {
                                'last_update': time.time(),
                                'satellites_used': len([s for s in report.get('satellites', []) if s.get('used', False)]),
                                'satellites_visible': len(report.get('satellites', [])),
                                'hdop': report.get('hdop', 0.0),
                                'vdop': report.get('vdop', 0.0),
                                'pdop': report.get('pdop', 0.0)
                            }
                    
                    # Check for NMEA data with RTK info
                    elif report.get('class') == 'NMEA' and 'device' in report:
                        device_path = report.get('device')
                        sentence = report.get('string', '')
                        
                        # Look for GGA sentence which contains RTK info
                        if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                            parts = sentence.split(',')
                            if len(parts) >= 7:
                                quality = parts[6]
                                if quality in ['4', '5']:  # RTK fix or float
                                    if device_path in self.devices:
                                        fix_mode = "RTK_FIX" if quality == '4' else "RTK_FLOAT"
                                        self.devices[device_path]['fix_mode'] = fix_mode
                                        self.devices[device_path]['rtk_capable'] = True
                                        self.devices[device_path]['fix_quality'] = int(quality)
                                        self.log(f"RTK mode detected from NMEA for {device_path}: {quality} → {fix_mode}")
                        
                except json.JSONDecodeError as e:
                    self.log(f"JSON error: {e} for line: {line[:50]}...")
                    
            return reports
        except socket.timeout:
            # No data received in timeout period (normal)
            return None
        except Exception as e:
            logger.error(f"Error processing GPSD data: {e}")
            return None
    
    def determine_fix_mode(self, report, mode, status):
        """Determine the detailed fix mode from various indicators"""
        
        # Start with basic mode mapping
        mode_map = {0: 'NO DATA', 1: 'NO FIX', 2: '2D FIX', 3: '3D FIX'}
        fix_mode = mode_map.get(mode, 'UNKNOWN')
        
        # Enhanced mode detection based on status
        status_map = {
            0: None,           # Unknown
            1: None,           # Normal, use basic mode
            2: "DGPS",         # Differential GPS
            3: "RTK_FIX",      # RTK Fixed solution
            4: "RTK_FLOAT",    # RTK Float solution
            5: "DR",           # Dead Reckoning
            6: "GNSS+DR",      # Combined GNSS and DR
            7: "TIME_ONLY",    # Time only
            8: "SIM",          # Simulation
            9: "SBAS"          # WAAS/EGNOS/MSAS
        }
        
        if status in status_map and status_map[status]:
            fix_mode = status_map[status]
        
        # Check for RTCM data, which indicates RTK corrections
        if 'rtcm' in report:
            self.log(f"RTCM data found in report")
            if fix_mode == '3D FIX':  # Only upgrade mode if already have basic 3D fix
                fix_mode = "RTK_DATA"  # Receiving RTK data but not yet applied
        
        # Check GGA sentence for RTK info (most reliable)
        if 'raw' in report:
            raw_data = report.get('raw', '')
            if '$GNGGA' in raw_data or '$GPGGA' in raw_data:
                gga_parts = raw_data.split(',')
                if len(gga_parts) >= 7:
                    quality = gga_parts[6]
                    quality_map = {
                        '0': 'NO FIX',        # Invalid
                        '1': 'GPS',           # GPS SPS
                        '2': 'DGPS',          # DGPS
                        '3': 'PPS',           # PPS
                        '4': 'RTK_FIX',       # RTK Fixed
                        '5': 'RTK_FLOAT',     # RTK Float
                        '6': 'ESTIMATED',     # Estimated/DR
                        '7': 'MANUAL',        # Manual
                        '8': 'SIMULATION',    # Simulation
                        '9': 'WAAS'           # WAAS/SBAS
                    }
                    if quality in quality_map:
                        fix_mode = quality_map[quality]
                        self.log(f"Mode from GGA quality: {quality} → {fix_mode}")
        
        return fix_mode
    
    def map_fix_quality(self, mode, status, fix_mode):
        """Map GPSD fix information to the fix_quality integer used by the rover"""
        if mode == 0:
            return 0  # No data
        if mode == 1:
            return 0  # No fix
        
        # Basic modes
        if mode == 2:
            return 1  # 2D fix
        if mode == 3:
            # Base case for 3D fix
            if fix_mode == "3D FIX":
                return 1
            # DGPS
            if status == 2 or fix_mode == "DGPS":
                return 2
            # SBAS/WAAS
            if status == 9 or fix_mode in ["SBAS", "WAAS"]:
                return 9
            # RTK float
            if status == 4 or fix_mode == "RTK_FLOAT":
                return 5
            # RTK fixed
            if status == 3 or fix_mode == "RTK_FIX":
                return 4
        
        # Default
        return 1
    
    def determine_device_roles(self):
        """Identify which device is rover (front) and which is base (rear)"""
        rover = None
        base = None
        unknown_devices = []
        
        for device_path, data in self.devices.items():
            # A device is likely a rover if it:
            # 1. Shows RTK_FIX or RTK_FLOAT mode
            # 2. Has RTCM data
            # 3. Has higher precision (lower eph)
            if (data.get('fix_mode') in ['RTK_FIX', 'RTK_FLOAT'] or 
                data.get('rtk_capable', False) or
                (data.get('eph', 99) < 0.5 and data.get('mode') == 3)):
                
                rover = device_path
                self.log(f"Identified rover: {device_path}")
                
            # Other devices with 3D fix are likely base stations
            elif data.get('mode') == 3:
                base = device_path
                self.log(f"Identified base: {device_path}")
            else:
                unknown_devices.append(device_path)
        
        return {
            'rover': rover,
            'base': base,
            'unknown': unknown_devices
        }
    
    def send_poll(self):
        """Send poll command to gpsd"""
        if self.socket:
            try:
                self.socket.send(b'?POLL;\n')
                return True
            except Exception as e:
                logger.warning(f"Error sending POLL command: {e}")
                return False
        return False


class GPSMonitor:
    """
    GPS monitor for tracking position and heading using dual GPS modules through GPSD.
    
    This implementation maintains the same interface as the serial-based GPSMonitor
    but uses GPSD for GPS data acquisition.
    """
    
    def __init__(self, front_port='/dev/ttyACM0', rear_port='/dev/ttyACM1', 
                 baud_rate=115200, timeout=0.1, simulation_mode=False,
                 gpsd_host='localhost', gpsd_port=2947):
        """
        Initialize GPS monitor that handles front and rear GPS units via GPSD.
        
        Args:
            front_port: Serial port for front GPS (used only for identification)
            rear_port: Serial port for rear GPS (used only for identification)
            baud_rate: Not used for GPSD
            timeout: Not used for GPSD
            simulation_mode: If True, generate simulated GPS data
            gpsd_host: GPSD host
            gpsd_port: GPSD port
        """
        self.simulation_mode = simulation_mode
        self.gpsd_host = gpsd_host
        self.gpsd_port = gpsd_port
        
        # For mapping between device paths and front/rear roles
        self.front_port = front_port
        self.rear_port = rear_port
        
        self.gps_data = {
            "front": self._create_empty_gps_data(),
            "rear": self._create_empty_gps_data()
        }
        
        logger.info(f"Initializing GPS Monitor using GPSD (simulation={simulation_mode})")
        
        # For real hardware mode
        if not simulation_mode:
            try:
                # Connect to GPSD
                self.gpsd_client = GpsdClient(host=gpsd_host, port=gpsd_port, debug=True)
                logger.info(f"Connected to GPSD at {gpsd_host}:{gpsd_port}")
                
                # Start GPSD reader thread
                self._start_gpsd_reader()
            except Exception as e:
                logger.error(f"Failed to initialize GPSD: {e}")
                logger.warning("Switching to simulation mode")
                self.simulation_mode = True
        
        # Start simulation if needed
        if self.simulation_mode:
            self._start_simulation()
        
        # Initialize last status update time
        self._last_status_log = 0
        
        # Initialize constants
        self.EARTH_RADIUS = 6371000  # Earth's radius in meters
    
    def _create_empty_gps_data(self):
        """Create an empty GPS data structure"""
        return {
            "lat": None,
            "lon": None, 
            "heading": None,
            "speed": 0.0,
            "fix_quality": 0,
            "last_update": 0,
            "messages_per_sec": 0,
            "hdop": 99.9,
            "satellites": 0,
            "device_path": None
        }
    
    def _start_gpsd_reader(self):
        """Start thread to read from GPSD"""
        self.running = True
        
        # Create and start GPSD reader thread
        self.gpsd_thread = threading.Thread(
            target=self._gpsd_reader,
            daemon=True
        )
        self.gpsd_thread.start()
    
    def _gpsd_reader(self):
        """Thread function to read and process GPSD data"""
        logger.debug("Starting GPSD reader thread")
        poll_count = 0
        
        try:
            while self.running:
                try:
                    # Poll periodically for data
                    if poll_count % 10 == 0:
                        self.gpsd_client.send_poll()
                    poll_count += 1
                    
                    # Process reports
                    self.gpsd_client.process_reports()
                    
                    # Determine device roles (rover vs base)
                    roles = self.gpsd_client.determine_device_roles()
                    
                    # Try to match devices to front/rear ports
                    front_device = None
                    rear_device = None
                    
                    # First check if we can identify by port name
                    for device_path in self.gpsd_client.devices:
                        if self.front_port in device_path:
                            front_device = device_path
                        elif self.rear_port in device_path:
                            rear_device = device_path
                    
                    # If we couldn't identify by port name, use rover/base roles
                    if not front_device and roles['rover']:
                        front_device = roles['rover']
                    if not rear_device and roles['base']:
                        rear_device = roles['base']
                    
                    # If we still have only one device, use it as front
                    if not front_device and not rear_device and self.gpsd_client.devices:
                        front_device = list(self.gpsd_client.devices.keys())[0]
                    
                    # Update front GPS data
                    if front_device and front_device in self.gpsd_client.devices:
                        device = self.gpsd_client.devices[front_device]
                        self.gps_data["front"]["device_path"] = front_device
                        self.gps_data["front"]["lat"] = device.get('lat')
                        self.gps_data["front"]["lon"] = device.get('lon')
                        self.gps_data["front"]["heading"] = device.get('track')
                        self.gps_data["front"]["speed"] = device.get('speed', 0.0)
                        self.gps_data["front"]["fix_quality"] = device.get('fix_quality', 0)
                        self.gps_data["front"]["last_update"] = device.get('last_update', 0)
                        self.gps_data["front"]["satellites"] = device.get('satellites_used', 0)
                        self.gps_data["front"]["hdop"] = device.get('hdop', 99.9)
                        self.gps_data["front"]["messages_per_sec"] = self._calculate_message_rate(front_device)
                    
                    # Update rear GPS data
                    if rear_device and rear_device in self.gpsd_client.devices:
                        device = self.gpsd_client.devices[rear_device]
                        self.gps_data["rear"]["device_path"] = rear_device
                        self.gps_data["rear"]["lat"] = device.get('lat')
                        self.gps_data["rear"]["lon"] = device.get('lon')
                        self.gps_data["rear"]["heading"] = device.get('track')
                        self.gps_data["rear"]["speed"] = device.get('speed', 0.0)
                        self.gps_data["rear"]["fix_quality"] = device.get('fix_quality', 0)
                        self.gps_data["rear"]["last_update"] = device.get('last_update', 0)
                        self.gps_data["rear"]["satellites"] = device.get('satellites_used', 0)
                        self.gps_data["rear"]["hdop"] = device.get('hdop', 99.9)
                        self.gps_data["rear"]["messages_per_sec"] = self._calculate_message_rate(rear_device)
                    
                    # Log debug info
                    now = time.time()
                    if now - self._last_status_log > 5.0:
                        self._log_gps_status()
                        self._last_status_log = now
                    
                    # Sleep a bit to avoid CPU overuse
                    time.sleep(0.1)
                    
                except Exception as e:
                    logger.error(f"Error in GPSD reader loop: {e}")
                    time.sleep(1)  # Avoid rapid error loops
                    
        except Exception as e:
            logger.error(f"Fatal error in GPSD reader thread: {e}")
    
    def _calculate_message_rate(self, device_path):
        """Calculate message rate for a device based on message count"""
        try:
            # Get current device data
            device = self.gpsd_client.devices.get(device_path)
            if not device:
                return 0
                
            # Get message count
            msg_count = device.get('messages_count', 0)
            last_update = device.get('last_update', 0)
            
            # Check if we have previous counts to compare
            if not hasattr(self, '_prev_message_counts'):
                self._prev_message_counts = {}
                self._prev_count_times = {}
            
            # Get previous count and time
            prev_count = self._prev_message_counts.get(device_path, 0)
            prev_time = self._prev_count_times.get(device_path, 0)
            
            # Calculate rate if we have previous data
            if prev_time > 0 and prev_count > 0:
                time_diff = last_update - prev_time
                if time_diff > 0:
                    count_diff = msg_count - prev_count
                    rate = count_diff / time_diff
                    
                    # Store current values for next calculation
                    self._prev_message_counts[device_path] = msg_count
                    self._prev_count_times[device_path] = last_update
                    
                    return rate
            
            # First run, just store values
            self._prev_message_counts[device_path] = msg_count
            self._prev_count_times[device_path] = last_update
            
            return 1.0  # Default value
            
        except Exception as e:
            logger.error(f"Error calculating message rate: {e}")
            return 0
    
    def _start_simulation(self):
        """Create a simple GPS simulation"""
        logger.info("Starting GPS simulation")
        
        # Default starting position in Wilmington
        self.gps_data["front"]["lat"] = 34.2257
        self.gps_data["front"]["lon"] = -77.9447
        self.gps_data["front"]["heading"] = 0.0
        self.gps_data["front"]["speed"] = 0.0
        self.gps_data["front"]["fix_quality"] = 4
        self.gps_data["front"]["satellites"] = 14
        self.gps_data["front"]["hdop"] = 0.8
        self.gps_data["front"]["last_update"] = time.time()
        
        # Rear GPS is 30cm behind
        self.gps_data["rear"]["lat"] = 34.2257 - 0.0000027
        self.gps_data["rear"]["lon"] = -77.9447
        self.gps_data["rear"]["heading"] = 0.0
        self.gps_data["rear"]["speed"] = 0.0
        self.gps_data["rear"]["fix_quality"] = 4
        self.gps_data["rear"]["satellites"] = 14
        self.gps_data["rear"]["hdop"] = 0.8
        self.gps_data["rear"]["last_update"] = time.time()
        
        # Set running flag
        self.running = True
        
        # Start simulation thread
        self.sim_thread = threading.Thread(
            target=self._sim_thread_function,
            daemon=True
        )
        self.sim_thread.start()
    
    def _simulate_gps_updates(self):
        """Simulate GPS updates based on speed and steering angle"""
        while self.simulation_running:
            try:
                time.sleep(self.simulation_update_interval)
                
                # Get current state
                current_speed = self.simulated_speed  # Meters per second
                current_steering_angle = self.simulated_steering_angle  # Radians
                
                # Calculate change in heading
                delta_heading = (current_speed / self.wheelbase) * np.tan(current_steering_angle) * self.simulation_update_interval
                
                # Update heading
                self.simulated_heading += np.degrees(delta_heading)
                self.simulated_heading %= 360  # Keep within 0-360 range
                
                # Calculate change in position (simplified for simulation)
                delta_x = current_speed * np.cos(np.radians(self.simulated_heading)) * self.simulation_update_interval
                delta_y = current_speed * np.sin(np.radians(self.simulated_heading)) * self.simulation_update_interval
                
                # IMPORTANT FIX: Use a more accurate scaling factor
                # Standard is 111111.0 meters per degree at the equator
                # But let's amplify this for better simulation progress
                SCALING_FACTOR = 11111.0  # 10x smaller = 10x faster movement
                
                # Update position (using enhanced increments in lat/lon)
                self.simulated_latitude += delta_y / SCALING_FACTOR  # Enhanced meters to latitude conversion
                self.simulated_longitude += (delta_x / (SCALING_FACTOR * np.cos(np.radians(self.simulated_latitude))))  # Enhanced meters to longitude
                
                # Log simulation update
                logger.debug(f"Simulated GPS update - Lat: {self.simulated_latitude:.8f}, Lon: {self.simulated_longitude:.8f}, Heading: {self.simulated_heading:.2f}")
                
            except Exception as e:
                logger.error(f"Error in GPS simulation: {e}")
    def _sim_thread_function(self):
        """Run GPS simulation updates"""
        # Constants for simulation
        update_interval = 0.1      # 10Hz update rate
        
        # Simulation state
        sim_speed = 0.0
        sim_heading = 0.0
        last_update = time.time()
        
        try:
            while self.running:
                now = time.time()
                delta_t = now - last_update
                
                if delta_t >= update_interval:
                    last_update = now
                    
                    # Simulate movement
                    distance = sim_speed * delta_t  # meters
                    
                    if distance > 0:
                        # Convert heading to radians for calculation
                        heading_rad = math.radians(sim_heading)
                        
                        # Get current position
                        front_lat = self.gps_data["front"]["lat"]
                        front_lon = self.gps_data["front"]["lon"]
                        
                        # Calculate new position using geodesic approach
                        # Use geopy's geodesic distance for accurate movement simulation
                        origin = (front_lat, front_lon)
                        
                        # Calculate destination based on bearing and distance
                        d = distance / 1000.0  # Convert to kilometers for this calculation
                        R = 6371.0  # Earth's radius in km
                        
                        lat1 = math.radians(front_lat)
                        lon1 = math.radians(front_lon)
                        
                        # Calculate new position (accurately)
                        lat2 = math.asin(math.sin(lat1) * math.cos(d/R) +
                                         math.cos(lat1) * math.sin(d/R) * math.cos(heading_rad))
                        lon2 = lon1 + math.atan2(math.sin(heading_rad) * math.sin(d/R) * math.cos(lat1),
                                               math.cos(d/R) - math.sin(lat1) * math.sin(lat2))
                        
                        # Convert back to degrees
                        new_lat = math.degrees(lat2)
                        new_lon = math.degrees(lon2)
                        
                        # Update front GPS position
                        self.gps_data["front"]["lat"] = new_lat
                        self.gps_data["front"]["lon"] = new_lon
                    
                    # Always update these properties
                    self.gps_data["front"]["heading"] = sim_heading
                    self.gps_data["front"]["speed"] = sim_speed
                    self.gps_data["front"]["last_update"] = now
                    self.gps_data["front"]["messages_per_sec"] = 10.0
                    
                    # Calculate rear GPS position (30cm behind front)
                    # For increased accuracy, calculate using the geodesic approach
                    rear_heading_rad = math.radians((sim_heading + 180) % 360)
                    rear_distance = 0.3 / 1000.0  # 30cm in kilometers
                    
                    # Front is our origin now
                    lat1 = math.radians(self.gps_data["front"]["lat"])
                    lon1 = math.radians(self.gps_data["front"]["lon"])
                    
                    # Calculate rear position
                    lat2 = math.asin(math.sin(lat1) * math.cos(rear_distance/R) +
                                    math.cos(lat1) * math.sin(rear_distance/R) * math.cos(rear_heading_rad))
                    lon2 = lon1 + math.atan2(math.sin(rear_heading_rad) * math.sin(rear_distance/R) * math.cos(lat1),
                                           math.cos(rear_distance/R) - math.sin(lat1) * math.sin(lat2))
                    
                    # Convert back to degrees
                    rear_lat = math.degrees(lat2)
                    rear_lon = math.degrees(lon2)
                    
                    # Update rear GPS position
                    self.gps_data["rear"]["lat"] = rear_lat
                    self.gps_data["rear"]["lon"] = rear_lon
                    self.gps_data["rear"]["heading"] = sim_heading
                    self.gps_data["rear"]["speed"] = sim_speed
                    self.gps_data["rear"]["last_update"] = now
                    self.gps_data["rear"]["messages_per_sec"] = 10.0
                
                # Sleep briefly to avoid CPU hogging
                time.sleep(0.01)
                
        except Exception as e:
            logger.error(f"Error in GPS simulation: {e}")
    
    def _log_gps_status(self):
        """Log GPS status for debugging"""
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        logger.debug("=== GPS STATUS ===")
        logger.debug(f"Front: ({front['lat']:.7f}, {front['lon']:.7f}) hdg={front['heading']}° spd={front['speed']:.1f}m/s fix={front['fix_quality']} sat={front['satellites']}")
        logger.debug(f"Rear: ({rear['lat']:.7f}, {rear['lon']:.7f}) hdg={rear['heading']}° spd={rear['speed']:.1f}m/s fix={rear['fix_quality']} sat={rear['satellites']}")
        
        # Calculate distances between GPSes
        if front["lat"] is not None and rear["lat"] is not None:
            distance = self.calculate_distance((rear["lat"], rear["lon"]), (front["lat"], front["lon"]))
            logger.debug(f"Distance between GPS units: {distance:.3f}m")
            
            # Calculate bearing using our method
            bearing = self.calculate_bearing(rear["lat"], rear["lon"], front["lat"], front["lon"])
            logger.debug(f"Calculated bearing between GPS units: {bearing:.1f}°")
        
        logger.debug("=================")
    
    def get_position_and_heading(self):
        """Get current position and heading (lat, lon, heading_deg, speed)"""
        if self.simulation_mode:
            # In simulation mode, just return simulation state
            heading_deg = np.degrees(self.sim_yaw) % 360
            return self.sim_x, self.sim_y, heading_deg, self.sim_speed
        else:
            # Use front GPS data for position
            lat = self.gps_data["front"]["lat"]
            lon = self.gps_data["front"]["lon"]
            
            # DIRECT IMPLEMENTATION: Always calculate heading from dual GPS if possible
            heading_deg = None  # Default to None
            
            # First attempt: Calculate from dual GPS positions (most accurate)
            if self.use_dual_gps and "rear" in self.gps_data:
                rear_lat = self.gps_data["rear"]["lat"]
                rear_lon = self.gps_data["rear"]["lon"]
                
                if (lat is not None and lon is not None and 
                    rear_lat is not None and rear_lon is not None):
                    # Calculate bearing from REAR to FRONT (vehicle heading)
                    heading_deg = self._calculate_bearing(rear_lat, rear_lon, lat, lon)
                    logger.debug(f"Dual GPS heading calculation: {heading_deg:.1f}°")
            
            # Second attempt: Use GPS-provided heading if dual calculation failed
            if heading_deg is None:
                heading_deg = self.gps_data["front"]["heading"]
                if heading_deg is not None:
                    logger.debug(f"Using direct GPS heading: {heading_deg:.1f}°")
                else:
                    heading_deg = 0.0
                    logger.warning("No heading data available, using 0.0°")
            
            # Get speed in m/s
            speed = self.gps_data["front"]["speed"] or 0.0
            
            return lat, lon, heading_deg, speed
    
    def stop(self):
        """Stop GPS monitoring"""
        if not self.simulation_mode:
            if hasattr(self, 'main_serial'):
                self.main_serial.close()
            if hasattr(self, 'secondary_serial') and self.use_dual_gps:
                self.secondary_serial.close()
    
    def update_simulation(self, steering_angle, speed, dt=0.1):
        """
        Update simulated position based on steering and speed
        Using a consistent bicycle model for simulation
        """
        # Ensure we're working with a copy of the state to prevent race conditions
        with self.lock:
            # Store previous state for logging
            prev_x = self.sim_x
            prev_y = self.sim_y
            prev_yaw = self.sim_yaw
            prev_heading_deg = np.degrees(prev_yaw) % 360
            
            # 1. Make sure steering_angle is in radians and limited
            steering_angle_rad = steering_angle
            if abs(steering_angle_rad) > np.pi/2:
                logger.warning(f"Very large steering angle detected: {np.degrees(steering_angle_rad):.1f}°, limiting to ±45°")
                steering_angle_rad = np.clip(steering_angle_rad, -np.pi/4, np.pi/4)
            
            # 2. Limit the yaw rate change for smoother simulation
            max_yaw_rate = np.radians(20)  # Max 20 degrees per update
            yaw_rate = (speed / self.L) * np.tan(steering_angle_rad)
            yaw_rate = np.clip(yaw_rate, -max_yaw_rate, max_yaw_rate)
            
            # 3. Update yaw (heading)
            self.sim_yaw += yaw_rate * dt
            self.sim_yaw = self.sim_yaw % (2 * np.pi)  # Keep in [0, 2π]
            
            # 4. Calculate position change
            # Using standard navigation coordinate system:
            # 0° = North, 90° = East, 180° = South, 270° = West
            heading_rad = self.sim_yaw
            delta_x = speed * np.sin(heading_rad) * dt  # East-West
            delta_y = speed * np.cos(heading_rad) * dt  # North-South
            
            # 5. Apply a scaling factor to make simulation progress visible
            # Standard conversion: 111,111 meters per degree latitude at equator
            # Using enhanced factor (10x) for better simulation visibility
            lat_scale = 1.0 / 11111.0  # 10x larger than real world
            lon_scale = 1.0 / (11111.0 * np.cos(np.radians(self.sim_x)))
            
            # 6. Convert to latitude/longitude changes
            lat_change = delta_y * lat_scale
            lon_change = delta_x * lon_scale
            
            # 7. Update position
            self.sim_x += lat_change
            self.sim_y += lon_change
            
            # 8. Update speed
            self.sim_speed = speed
            
            # 9. Comprehensive logging
            heading_deg = np.degrees(self.sim_yaw) % 360
            heading_change = ((heading_deg - prev_heading_deg + 180) % 360) - 180  # -180 to +180
            
            logger.info(
                f"SIMULATION: Speed={speed:.2f}m/s, Steering={np.degrees(steering_angle_rad):.1f}°\n"
                f"Position: ({prev_x:.6f}, {prev_y:.6f}) → ({self.sim_x:.6f}, {self.sim_y:.6f})\n"
                f"Heading: {prev_heading_deg:.1f}° → {heading_deg:.1f}° (Δ={heading_change:.1f}°)\n"
                f"Movement: N/S={delta_y:.2f}m, E/W={delta_x:.2f}m"
            )
    
def run_gps_test(waypoints_file, config=None):
    # ... existing code ...
    
    while True:
        # ... existing code ...
        
        # Get GPS data
        lat, lon, heading_rad, speed = gps.get_position_and_heading()
        
        # CRITICAL FIX: Convert heading from radians back to degrees
        heading_deg = np.degrees(heading_rad) % 360
        
        # Print GPS information
        logger.info(f"\n===== GPS DIAGNOSTICS =====")
        logger.info(f"Position: {lat:.8f}, {lon:.8f}")
        logger.info(f"GPS Heading: {heading_deg:.1f}° | Speed: {speed:.2f} m/s")

def run_hardware(waypoints_file, config=None):
    # ... existing code ...
    
    # Create GPS monitor  
    gps = GPSMonitor(simulation_mode=False, use_dual_gps=True)
    
    # Apply the heading fix to ensure consistency

    
    # ... rest of the function ...

def _add_simulation_diagnostics():
    """Add debug output to main.py to display simulation diagnostics"""
    # In run_simulation before applying to hardware/simulation:
    
    # Display simulation diagnostics
    waypoint_lat, waypoint_lon = controller.waypoints[target_idx]
    bearing_to_wp = controller._calculate_bearing(x, y, waypoint_lat, waypoint_lon)
    
    logger.info(
        f"\n--- SIMULATION DIAGNOSTICS ---\n"
        f"Current position: ({x:.6f}, {y:.6f})\n"
        f"Current heading: {np.degrees(yaw):.1f}° (raw yaw: {yaw:.4f} rad)\n"
        f"Target waypoint {target_idx+1}/{len(waypoints)}: ({waypoint_lat:.6f}, {waypoint_lon:.6f})\n"
        f"Bearing to waypoint: {bearing_to_wp:.1f}°\n"
        f"Distance: {distance:.2f}m, CTE: {cte:.2f}m\n"
        f"Heading error: {np.degrees(yaw_error):.1f}°\n"
        f"Steering command: {np.degrees(delta):.1f}°\n"
        f"Speed: {v:.2f}m/s\n"
        f"---------------------------"
    )
    
    # After gps.update_simulation(), add:
    new_x, new_y, new_heading, _ = gps.get_position_and_heading()
    logger.info(
        f"Position updated: ({x:.6f}, {y:.6f}) → ({new_x:.6f}, {new_y:.6f})\n"
        f"Heading updated: {np.degrees(yaw):.1f}° → {new_heading:.1f}°"
    )

# For testing
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
        """Get the current position (lat, lon) and heading from the GPS devices"""
        # In simulation mode, return simulated data
        if self.simulation_mode:
            front = self.gps_data["front"]
            rear = self.gps_data["rear"]
            
            # Calculate heading between GPSes
            if None not in (front["lat"], front["lon"], rear["lat"], rear["lon"]):
                calculated_heading = self.calculate_bearing(
                    rear["lat"], rear["lon"],
                    front["lat"], front["lon"]
                )
            else:
                calculated_heading = front["heading"]
                
            return front["lat"], front["lon"], calculated_heading, front["speed"]
        
        # For GPSD mode, use the GPS data we've collected
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        lat, lon = None, None
        heading = None
        speed = 0.0
        
        # First, try to get position from front GPS
        if front["lat"] is not None and front["lon"] is not None:
            lat = front["lat"]
            lon = front["lon"]
            speed = front["speed"]
            
            # Try to get heading from the dual-GPS setup
            if rear["lat"] is not None and rear["lon"] is not None:
                # Calculate heading from dual GPS positions
                heading = self.calculate_bearing(
                    rear["lat"], rear["lon"],
                    front["lat"], front["lon"]
                )
                logger.debug("Heading calculated from dual GPS positions")
            
            # If dual-GPS heading failed, try using NMEA heading data
            if heading is None and front["heading"] is not None:
                heading = front["heading"]
                logger.debug("Using NMEA heading from front GPS")
        
        # If still no heading and we have multiple positions in history, try calculating from movement
        if heading is None and hasattr(self, '_last_positions') and len(self._last_positions) >= 2:
            try:
                # Get two most recent positions
                p1, t1 = self._last_positions[-2]
                p2, t2 = self._last_positions[-1]
                
                # Only calculate if positions are different enough
                distance = self.calculate_distance(p1, p2)
                if distance > 0.2:  # More than 20cm
                    heading = self.calculate_bearing(p1[0], p1[1], p2[0], p2[1])
                    logger.debug(f"Heading calculated from movement history: {heading}°")
            except Exception as e:
                logger.warning(f"Failed to calculate heading from history: {e}")
        
        # Store position history for movement-based heading calculation
        timestamp = time.time()
        if not hasattr(self, '_last_positions'):
            self._last_positions = []
        
        # Store current position if valid
        if lat is not None and lon is not None:
            self._last_positions.append(((lat, lon), timestamp))
            
            # Keep only recent positions
            MAX_POSITIONS = 10
            if len(self._last_positions) > MAX_POSITIONS:
                self._last_positions = self._last_positions[-MAX_POSITIONS:]
        
        return lat, lon, heading, speed
    
    def get_gps_precision(self):
        """Get the precision estimates for the GPS units"""
        if self.simulation_mode:
            # Return simulated precision
            return {
                "front": {"horizontal": 0.01, "vertical": 0.02},
                "rear": {"horizontal": 0.01, "vertical": 0.02}
            }
        
        # For GPSD mode, get precision from the collected data
        precision = {
            "front": {
                "horizontal": self.gpsd_client.devices.get(
                    self.gps_data["front"].get("device_path", ""), {}).get('eph', 99.9),
                "vertical": self.gpsd_client.devices.get(
                    self.gps_data["front"].get("device_path", ""), {}).get('epv', 99.9)
            },
            "rear": {
                "horizontal": self.gpsd_client.devices.get(
                    self.gps_data["rear"].get("device_path", ""), {}).get('eph', 99.9),
                "vertical": self.gpsd_client.devices.get(
                    self.gps_data["rear"].get("device_path", ""), {}).get('epv', 99.9)
            }
        }
        
        return precision
    
    def get_fix_quality(self):
        """Get the fix quality (0-9) for each GPS"""
        return {
            "front": self.gps_data["front"]["fix_quality"],
            "rear": self.gps_data["rear"]["fix_quality"]
        }
    
    def get_satellite_count(self):
        """Get the number of satellites visible to each GPS"""
        return {
            "front": self.gps_data["front"]["satellites"],
            "rear": self.gps_data["rear"]["satellites"]
        }
    
    def get_message_rate(self):
        """Get the message rate in messages/second for each GPS"""
        return {
            "front": self.gps_data["front"]["messages_per_sec"],
            "rear": self.gps_data["rear"]["messages_per_sec"]
        }
    
    def get_raw_gps_data(self):
        """Get the full GPS data structure for debugging"""
        return self.gps_data
    
    def calculate_distance(self, point1, point2):
        """Calculate distance between two GPS coordinates in meters"""
        try:
            # Use geodesic distance for accurate calculations
            return geodesic(point1, point2).meters
        except Exception as e:
            logger.error(f"Error calculating distance: {e}")
            return 0
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing angle from point1 to point2"""
        try:
            # Convert to radians
            lat1_rad = math.radians(lat1)
            lon1_rad = math.radians(lon1)
            lat2_rad = math.radians(lat2)
            lon2_rad = math.radians(lon2)
            
            # Calculate bearing
            y = math.sin(lon2_rad - lon1_rad) * math.cos(lat2_rad)
            x = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(lon2_rad - lon1_rad)
            
            bearing = math.atan2(y, x)
            bearing = math.degrees(bearing)
            bearing = (bearing + 360) % 360  # Convert to 0-360 range
            
            return bearing
            
        except Exception as e:
            logger.error(f"Error calculating bearing: {e}")
            return None
    
    def get_distance_between_gps(self):
        """Calculate the distance between front and rear GPS units in meters"""
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        if None in (front["lat"], front["lon"], rear["lat"], rear["lon"]):
            return None
            
        return self.calculate_distance((front["lat"], front["lon"]), (rear["lat"], rear["lon"]))
    
    def shutdown(self):
        """Clean up and close connections"""
        logger.info("Shutting down GPS Monitor")
        self.running = False
        
        # Close GPSD connection if existing
        if hasattr(self, 'gpsd_client') and self.gpsd_client and self.gpsd_client.socket:
            try:
                self.gpsd_client.socket.close()
                logger.debug("Closed GPSD socket")
            except Exception as e:
                logger.error(f"Error closing GPSD socket: {e}")

# For testing the module directly
if __name__ == "__main__":
    # Set up logging
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create GPS monitor instance
    gps = GPSMonitor(simulation_mode=False)
    
    try:
        # Print GPS data in a loop
        print("Monitoring GPS data. Press Ctrl+C to exit.")
        while True:
            lat, lon, heading_rad, speed = gps.get_position_and_heading()
            
            # CRITICAL FIX: Convert heading from radians to degrees
            heading_deg = np.degrees(heading_rad) % 360
            
            print(f"Position: {lat:.7f}, {lon:.7f} | Heading: {heading_deg:.1f}° | Speed: {speed:.1f} m/s")
            
            # Debug: Show raw values to confirm
            print(f"DEBUG - Raw heading in radians: {heading_rad:.4f}, converted to deg: {heading_deg:.1f}°")
            
            # If using dual GPS, show values for verification
            if not gps.simulation_mode and gps.use_dual_gps:
                front_lat = gps.gps_data["front"]["lat"]
                front_lon = gps.gps_data["front"]["lon"]
                rear_lat = gps.gps_data["rear"]["lat"]
                rear_lon = gps.gps_data["rear"]["lon"]
                if all(p is not None for p in [front_lat, front_lon, rear_lat, rear_lon]):
                    direct_heading = gps._calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
                    print(f"DEBUG - DIRECT: Rear to Front heading: {direct_heading:.1f}°")
            lat, lon, heading, speed = gps.get_position_and_heading()
            precision = gps.get_gps_precision()
            fix_quality = gps.get_fix_quality()
            satellites = gps.get_satellite_count()
            distance = gps.get_distance_between_gps()
            
            print("\n=== GPS Status ===")
            print(f"Position: {lat:.7f}, {lon:.7f}")
            print(f"Heading: {heading}°, Speed: {speed:.2f} m/s")
            print(f"Fix Quality: Front={fix_quality['front']}, Rear={fix_quality['rear']}")
            print(f"Satellites: Front={satellites['front']}, Rear={satellites['rear']}")
            if distance:
                print(f"GPS Separation: {distance:.3f}m")
            print(f"Precision (m): Front H±{precision['front']['horizontal']:.3f} V±{precision['front']['vertical']:.3f}")
            print("=================")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        gps.shutdown()