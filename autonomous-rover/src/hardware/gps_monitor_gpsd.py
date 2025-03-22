#!/usr/bin/env python3

import time
import math
import threading
import logging
import socket
import json
from geopy.distance import geodesic

# Configure logging
logger = logging.getLogger("rover.hardware.gpsd")

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
        """Connect to GPSD service with improved multi-device detection"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            logger.info(f"✓ Connected to GPSD at {self.host}:{self.port}")
            
            # Enhanced watch command with full features
            watch_command = b'?WATCH={"enable":true,"json":true,"nmea":true,"raw":1,"scaled":true,"device":true};\n'
            self.socket.send(watch_command)
            
            # Initialize buffer if not already set
            if not hasattr(self, 'buffer'):
                self.buffer = ""
            
            # Wait for initial response
            time.sleep(0.5)
            
            # Set default expected devices if front_port and rear_port aren't explicitly set
            if not hasattr(self, 'front_port'):
                self.front_port = '/dev/ttyACM0'
            if not hasattr(self, 'rear_port'):
                self.rear_port = '/dev/ttyACM1'
                
            # Expected devices
            expected_devices = [self.front_port, self.rear_port]
            logger.info(f"Looking for GPS devices: {expected_devices}")
            
            # Deep device discovery
            discovery_attempts = 12  # Increased attempts for thoroughness
            
            for i in range(discovery_attempts):
                logger.info(f"Discovery attempt {i+1}/{discovery_attempts}...")
                
                # Send a variety of commands based on iteration
                if i % 3 == 0:
                    # Request device list
                    self.socket.send(b'?DEVICES;\n')
                elif i % 3 == 1:
                    # Poll with specific device requests for each expected device
                    for device in expected_devices:
                        self.socket.send(f'?WATCH={{"enable":true,"json":true,"nmea":true,"raw":1,"device":"{device}"}};\n'.encode())
                        time.sleep(0.2)
                else:
                    # General poll
                    self.socket.send(b'?POLL;\n')
                
                # Process with longer timeout on later attempts
                timeout = min(1.0 + (i * 0.5), 3.0)
                self.process_reports(timeout=timeout)
                
                # Check if we found the devices we need
                if all(device in self.devices for device in expected_devices):
                    logger.info("✅ SUCCESS: Found all expected GPS devices!")
                    break
                    
                # Or at least found something
                if len(self.devices) >= 2:
                    logger.info(f"✅ Found {len(self.devices)} GPS devices (expected {len(expected_devices)})")
                    break
                    
                # Report on missing devices
                missing_devices = [dev for dev in expected_devices if dev not in self.devices]
                if missing_devices:
                    logger.info(f"Still missing devices: {', '.join(missing_devices)}")
                    
                    # Try specific device watch
                    for device in missing_devices:
                        device_watch = f'?WATCH={{"enable":true,"json":true,"nmea":true,"raw":1,"device":"{device}"}};\n'
                        self.socket.send(device_watch.encode())
                        logger.info(f"  Sent explicit watch for {device}")
                        time.sleep(0.8)
                        self.process_reports(timeout=1.5)
                
                # Sleep between attempts
                time.sleep(0.5)
            
            # Report final device discovery status
            if len(self.devices) == 0:
                logger.error("❌ ERROR: No GPS devices found after multiple attempts")
                return False
            elif len(self.devices) == 1:
                logger.warning("⚠️ WARNING: Only found one GPS device")
            else:
                logger.info(f"✓ Found {len(self.devices)} GPS devices")
                
            # Log discovered devices
            for device_path, info in self.devices.items():
                model = "unknown"
                if 'subtype1' in info:
                    subtype = info.get('subtype1', '')
                    if 'MOD=' in subtype:
                        import re
                        match = re.search(r'MOD=([^,]+)', subtype)
                        if match:
                            model = match.group(1)
                            
                logger.info(f"  - {device_path}: {model} with {info.get('satellites_used', 0)} satellites")
            
            # If we still don't have enough devices, diagnose the problem
            if len(self.devices) < 2:
                self._diagnose_missing_devices(expected_devices)
                
                # For this module, we must have at least two devices
                if len(self.devices) < 2:
                    logger.error("Cannot continue without at least two GPS devices")
                    return False
                
            return True
        except Exception as e:
            logger.error(f"Error connecting to GPSD: {e}")
            import traceback
            traceback.print_exc()
            return False

    def _diagnose_missing_devices(self, expected_devices):
        """Diagnose why devices are missing"""
        logger.info("\n=== DIAGNOSING MISSING GPS DEVICES ===")
        
        try:
            # Check if device files exist
            logger.info("Checking if device files exist...")
            for device in expected_devices:
                if os.path.exists(device):
                    logger.info(f"  ✓ Device file exists: {device}")
                    
                    # Check permissions
                    try:
                        import stat, os, pwd
                        st = os.stat(device)
                        perms = stat.filemode(st.st_mode)
                        user = pwd.getpwuid(st.st_uid).pw_name
                        logger.info(f"    Permissions: {perms} (owner: {user})")
                        
                        # Check if current user can access
                        import getpass
                        current_user = getpass.getuser()
                        if user != current_user and 'dialout' not in os.popen(f"groups {current_user}").read():
                            logger.warning(f"    Current user ({current_user}) may not have permission to access {device}")
                            logger.warning(f"    Try: sudo usermod -a -G dialout {current_user}")
                    except:
                        pass
                    
                    # Try to get more device info
                    try:
                        import subprocess
                        cmd = f"udevadm info -a -n {device} | grep -E 'idVendor|idProduct|manufacturer|product' | head -4"
                        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                        if result.stdout:
                            for line in result.stdout.splitlines():
                                logger.info(f"    {line.strip()}")
                    except:
                        pass
                    
                    # Check if device is in use
                    try:
                        import subprocess
                        cmd = f"lsof {device}"
                        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                        if result.stdout:
                            logger.info(f"    Device is in use by:")
                            for line in result.stdout.splitlines():
                                logger.info(f"      {line}")
                        else:
                            logger.info(f"    Device is not in use by other processes")
                    except:
                        pass
                else:
                    logger.error(f"  ✗ Device file does not exist: {device}")
                    
            # Check GPSD configuration
            try:
                logger.info("\nChecking GPSD configuration:")
                if os.path.exists('/etc/default/gpsd'):
                    with open('/etc/default/gpsd', 'r') as f:
                        config = f.read()
                        logger.info(config)
                        
                # Check GPSD processes
                logger.info("\nActive GPSD processes:")
                import subprocess
                ps = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
                for line in ps.stdout.splitlines():
                    if 'gpsd' in line and 'grep' not in line:
                        logger.info(f"  {line}")
            except Exception as e:
                logger.warning(f"Could not check GPSD configuration: {e}")
                
            # Try direct device access if device exists but isn't showing in GPSD
            for device in expected_devices:
                if device not in self.devices and os.path.exists(device):
                    logger.info(f"\nAttempting direct access to {device}...")
                    try:
                        # Try direct serial read
                        import serial
                        ser = serial.Serial(device, 115200, timeout=1)
                        logger.info(f"  Opened {device} directly")
                        
                        for i in range(5):
                            line = ser.readline().decode('ascii', errors='replace').strip()
                            if line:
                                logger.info(f"  Read: {line}")
                        ser.close()
                        
                        # Suggest manual gpsd restart
                        logger.info("  Suggesting manual gpsd restart:")
                        logger.info(f"    sudo systemctl stop gpsd")
                        logger.info(f"    sudo gpsd -n {' '.join(expected_devices)}")
                        logger.info(f"    sudo systemctl start gpsd")
                        
                    except ImportError:
                        logger.warning("  pyserial not installed, can't test direct access")
                    except Exception as e:
                        logger.warning(f"  Direct access failed: {e}")
        except Exception as e:
            logger.error(f"Error diagnosing missing devices: {e}")

    def process_reports(self, timeout=1.0):
        """Process incoming GPSD reports with improved JSON parsing for mixed binary/JSON data"""
        if not self.socket:
            return None
            
        try:
            # Use a shorter timeout to process data chunks
            self.socket.settimeout(timeout)
            
            try:
                # Receive data
                data = self.socket.recv(8192).decode('utf-8', errors='replace')
                if not data:
                    return None
            except socket.timeout:
                # No data available
                return None
                
            self.log(f"Received {len(data)} bytes of data")
            
            # Add to the buffer for handling partial JSON
            if not hasattr(self, 'buffer'):
                self.buffer = ""
            self.buffer += data
            
            # Process complete JSON objects
            reports = []
            
            while True:
                # Find start of a JSON object
                start_idx = self.buffer.find('{')
                if start_idx == -1:
                    self.buffer = ""  # No JSON start found
                    break
                    
                # Find the end of the JSON object by matching braces
                end_idx = -1
                depth = 0
                
                for i in range(start_idx, len(self.buffer)):
                    if self.buffer[i] == '{':
                        depth += 1
                    elif self.buffer[i] == '}':
                        depth -= 1
                        if depth == 0:
                            end_idx = i
                            break
                            
                if end_idx == -1:
                    # No complete JSON object found
                    break
                    
                # Extract complete JSON object
                json_str = self.buffer[start_idx:end_idx + 1]
                
                # Find where to continue parsing
                semicolon_idx = self.buffer.find(';', end_idx)
                if semicolon_idx != -1:
                    self.buffer = self.buffer[semicolon_idx+1:]
                else:
                    self.buffer = self.buffer[end_idx+1:]
                    
                # Parse the JSON
                try:
                    report = json.loads(json_str)
                    reports.append(report)
                    
                    # Process the report based on class
                    if report.get('class') == 'TPV' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Get extended information
                        status = report.get('status', 0)
                        mode = report.get('mode', 0)
                        
                        # Determine the complete fix mode
                        fix_mode = self.determine_fix_mode(report, mode, status)
                        
                        # Store or update device info
                        if device_path not in self.devices:
                            self.devices[device_path] = {}
                        
                        # Update the device with position data
                        self.devices[device_path].update({
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
                        })
                        
                        self.log(f"Updated TPV for {device_path} with fix mode {fix_mode}")
                    
                    # Process SKY reports for satellite info
                    elif report.get('class') == 'SKY' and 'device' in report:
                        device_path = report.get('device')
                        
                        # Initialize device entry if needed
                        if device_path not in self.devices:
                            self.devices[device_path] = {'last_update': time.time()}
                        
                        # Calculate satellites used count
                        sats_used = sum(1 for s in report.get('satellites', []) if s.get('used', False))
                        sats_visible = len(report.get('satellites', []))
                        
                        # Update device info with satellite data
                        self.devices[device_path].update({
                            'satellites_used': sats_used,
                            'satellites_visible': sats_visible,
                            'hdop': report.get('hdop', 0.0),
                            'vdop': report.get('vdop', 0.0),
                            'pdop': report.get('pdop', 0.0)
                        })
                        
                        self.log(f"Updated SKY for {device_path}: {sats_used}/{sats_visible} satellites")
                    
                    # Extract device info from DEVICES report
                    elif report.get('class') == 'DEVICES':
                        devices_list = report.get('devices', [])
                        self.log(f"Received DEVICES report with {len(devices_list)} devices")
                        
                        for device in devices_list:
                            path = device.get('path')
                            if not path:
                                continue
                                
                            # Create new entry if needed
                            if path not in self.devices:
                                self.devices[path] = {'last_update': time.time()}
                            
                            # Update device info
                            self.devices[path].update({
                                'driver': device.get('driver', 'unknown'),
                                'subtype': device.get('subtype', ''),
                                'subtype1': device.get('subtype1', ''),
                                'activated': device.get('activated', ''),
                                'native': device.get('native', 0) == 1
                            })
                            
                            self.log(f"Added/updated device: {path}")
                    
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
                                    # Initialize device entry if needed
                                    if device_path not in self.devices:
                                        self.devices[device_path] = {'last_update': time.time()}
                                    
                                    fix_mode = "RTK_FIX" if quality == '4' else "RTK_FLOAT"
                                    self.devices[device_path].update({
                                        'fix_mode': fix_mode,
                                        'rtk_capable': True,
                                        'fix_quality': int(quality)
                                    })
                                    self.log(f"RTK mode detected from NMEA for {device_path}: {quality} → {fix_mode}")
                    
                    # VERSION report
                    elif report.get('class') == 'VERSION':
                        self.log(f"GPSD version: {report.get('release', 'unknown')}")
                    
                except json.JSONDecodeError as e:
                    self.log(f"JSON error: {e} for: {json_str[:50]}...")
                except Exception as e:
                    self.log(f"Error processing report: {e}")
                    
            return reports
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
        
        # Log all available devices first
        logger.info(f"GPSD reports {len(self.devices)} connected GPS devices")
        for device_path, data in self.devices.items():
            fix_type = data.get('fix_mode', 'UNKNOWN')
            satellites = data.get('satellites_used', 0)
            logger.info(f"Device {device_path}: {fix_type} fix with {satellites} satellites")
        
        # First try to identify by exact port names
        for device_path in self.devices:
            if self.front_port in device_path:
                rover = device_path
                logger.info(f"Front GPS identified by port name: {device_path}")
            elif self.rear_port in device_path:
                base = device_path
                logger.info(f"Rear GPS identified by port name: {device_path}")
        
        # If we haven't found both, use fix quality and other metrics
        if not rover or not base:
            best_rover = None
            best_rover_score = -1
            best_base = None
            best_base_score = -1
            
            for device_path, data in self.devices.items():
                # Skip devices we've already assigned
                if device_path == rover or device_path == base:
                    continue
                
                # Calculate a "quality score" for device ranking
                fix_mode = data.get('fix_mode', 'UNKNOWN')
                fix_quality = data.get('fix_quality', 0)
                eph = data.get('eph', 99.9)
                satellites = data.get('satellites_used', 0)
                
                # Higher score = better candidate
                score = satellites * 10  # Satellites are most important
                
                # Add quality bonus
                if fix_quality >= 4:  # RTK fixed
                    score += 1000
                elif fix_quality >= 2:  # DGPS or better
                    score += 500
                elif fix_quality >= 1:  # 3D fix
                    score += 100
                    
                # Better precision = higher score
                if eph < 1.0:
                    score += 200
                elif eph < 2.0:
                    score += 100
                
                # If not already identified as rover or base, assign based on score
                if not rover and (best_rover is None or score > best_rover_score):
                    best_rover = device_path
                    best_rover_score = score
                elif not base and rover != device_path and (best_base is None or score > best_base_score):
                    best_base = device_path
                    best_base_score = score
            
            # Assign best candidates if needed
            if not rover and best_rover:
                rover = best_rover
                logger.info(f"Front GPS identified by quality score ({best_rover_score}): {rover}")
            
            if not base and best_base:
                base = best_base
                logger.info(f"Rear GPS identified by quality score ({best_base_score}): {base}")
        
        # If we still only have one device, make sure it's assigned as front
        if not base and not rover and self.devices:
            rover = list(self.devices.keys())[0]
            logger.info(f"Only one GPS found, assigning as front: {rover}")
        
        # Final fallback - if we have two devices but haven't assigned both roles,
        # assign the remaining device to the missing role
        if rover and not base and len(self.devices) >= 2:
            for device in self.devices:
                if device != rover:
                    base = device
                    logger.info(f"Assigned remaining device as rear GPS: {base}")
                    break
        
        if base and not rover and len(self.devices) >= 2:
            for device in self.devices:
                if device != base:
                    rover = device
                    logger.info(f"Assigned remaining device as front GPS: {rover}")
                    break
        
        # Report the results
        if rover:
            logger.info(f"Front GPS assigned to: {rover}")
        else:
            logger.warning("No front GPS identified!")
            
        if base:
            logger.info(f"Rear GPS assigned to: {base}")
        else:
            logger.warning("No rear GPS identified!")
        
        return {
            'rover': rover,
            'base': base,
            'unknown': [d for d in self.devices if d != rover and d != base]
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
    """Monitor for GPS devices using GPSD"""
    
    def __init__(self, front_port='/dev/ttyACM0', rear_port='/dev/ttyACM1', 
                 baud_rate=115200, timeout=0.1, simulation_mode=False,
                 gpsd_host='localhost', gpsd_port=2947):
        # For mapping between device paths and front/rear roles
        self.front_port = front_port
        self.rear_port = rear_port
        
        # Initialize the running flag to fix thread error
        self.running = True  # Add this line
        
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
                
                # Pass port info to client BEFORE connecting
                self.gpsd_client.front_port = front_port  # Set this before connect()
                self.gpsd_client.rear_port = rear_port    # Set this before connect()
                
                # Check if we successfully connected and found devices
                if not self.gpsd_client.connect():
                    logger.error("Failed to connect to GPSD or find required GPS devices")
                    logger.warning("Switching to simulation mode")
                    simulation_mode = True
                else:
                    logger.info(f"Connected to GPSD at {gpsd_host}:{gpsd_port}")
                    
                    # Start GPSD reader thread
                    self._start_gpsd_reader()
                    
                    # Start the device monitoring thread
                    self._start_gps_monitor_thread()
                    
            except Exception as e:
                logger.error(f"Failed to initialize GPSD: {e}")
                logger.warning("Switching to simulation mode")
                simulation_mode = True
        
        # Set simulation mode
        self.simulation_mode = simulation_mode
        
        # Start simulation if needed
        if self.simulation_mode:
            self._start_simulation()
        
        # Initialize last status update time
        self._last_status_log = 0
        
        # Initialize constants
        self.EARTH_RADIUS = 6371000  # Earth's radius in meters
        
        # Log initial status
        if not simulation_mode:
            logger.info(f"GPS Monitor initialized with front port={front_port}, rear port={rear_port}")

    def _discover_gps_devices_with_timeout(self, max_wait=5.0):
        """Initial discovery of GPS devices with timeout, returns True if both devices found"""
        logger.info(f"Discovering GPS devices (timeout: {max_wait}s)...")
        
        start_time = time.time()
        front_found = False
        rear_found = False
        
        while time.time() - start_time < max_wait:
            # Send poll command and process reports
            self.gpsd_client.send_poll()
            time.sleep(0.2)
            self.gpsd_client.socket.send(b'?DEVICES;\n')
            time.sleep(0.3)
            self.gpsd_client.process_reports()
            
            # Check if we found devices
            if self.gpsd_client.devices:
                # Look for front and rear devices
                for device_path in self.gpsd_client.devices:
                    if self.front_port in device_path:
                        front_found = True
                        self._update_front_gps_data(device_path)
                    elif self.rear_port in device_path:
                        rear_found = True
                        self._update_rear_gps_data(device_path)
                
                # If we found both, we can exit early
                if front_found and rear_found:
                    logger.info("✅ Found both GPS devices!")
                    break
            
            # Display progress
            elapsed = time.time() - start_time
            devices_found = len(self.gpsd_client.devices)
            logger.debug(f"Discovery: {elapsed:.1f}s elapsed, found {devices_found} device(s)")
            time.sleep(0.5)
        
        # If we didn't find both GPSes by device name, try to use what we found
        if not (front_found and rear_found):
            # Run device role determination
            roles = self.gpsd_client.determine_device_roles()
            
            # Check if we got rover/base assignments
            if roles['rover'] and not front_found:
                front_found = True
                self._update_front_gps_data(roles['rover'])
                logger.info(f"Using rover device as front GPS: {roles['rover']}")
                
            if roles['base'] and not rear_found:
                rear_found = True
                self._update_rear_gps_data(roles['base'])
                logger.info(f"Using base device as rear GPS: {roles['base']}")
        
        # Check if we found two devices and they're different
        devices = list(self.gpsd_client.devices.keys())
        if len(devices) >= 2 and not (front_found and rear_found):
            # Assign the first two devices to front and rear
            if not front_found:
                front_found = True
                self._update_front_gps_data(devices[0])
                logger.info(f"Assigned first available device as front GPS: {devices[0]}")
                
            if not rear_found and devices[0] != devices[1]:
                rear_found = True
                self._update_rear_gps_data(devices[1])
                logger.info(f"Assigned second available device as rear GPS: {devices[1]}")
        
        # Log final discovery status
        if front_found and rear_found:
            logger.info(f"Successfully found both front and rear GPS units")
            return True
        else:
            logger.warning(f"Failed to find both GPS units: Front={front_found}, Rear={rear_found}")
            return False

    def _update_front_gps_data(self, device_path):
        """Update front GPS data from the given device"""
        if device_path and device_path in self.gpsd_client.devices:
            device = self.gpsd_client.devices[device_path]
            self.gps_data["front"]["device_path"] = device_path
            self.gps_data["front"]["lat"] = device.get('lat')
            self.gps_data["front"]["lon"] = device.get('lon')
            self.gps_data["front"]["heading"] = device.get('track')
            self.gps_data["front"]["speed"] = device.get('speed', 0.0)
            self.gps_data["front"]["fix_quality"] = device.get('fix_quality', 0)
            self.gps_data["front"]["last_update"] = device.get('last_update', 0)
            self.gps_data["front"]["satellites"] = device.get('satellites_used', 0)
            self.gps_data["front"]["hdop"] = device.get('hdop', 99.9)
            logger.info(f"Updated front GPS data from {device_path}")

    def _update_rear_gps_data(self, device_path):
        """Update rear GPS data from the given device"""
        if device_path and device_path in self.gpsd_client.devices:
            device = self.gpsd_client.devices[device_path]
            self.gps_data["rear"]["device_path"] = device_path
            self.gps_data["rear"]["lat"] = device.get('lat')
            self.gps_data["rear"]["lon"] = device.get('lon')
            self.gps_data["rear"]["heading"] = device.get('track')
            self.gps_data["rear"]["speed"] = device.get('speed', 0.0)
            self.gps_data["rear"]["fix_quality"] = device.get('fix_quality', 0)
            self.gps_data["rear"]["last_update"] = device.get('last_update', 0)
            self.gps_data["rear"]["satellites"] = device.get('satellites_used', 0)
            self.gps_data["rear"]["hdop"] = device.get('hdop', 99.9)
            logger.info(f"Updated rear GPS data from {device_path}")
    
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
        # Clear the screen first

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
        
        # For GPSD mode, ALWAYS recalculate heading from GPS positions if we have both
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        # Check if we have valid positions from both GPS units
        both_gps_valid = (
            front["lat"] is not None and front["lon"] is not None and
            rear["lat"] is not None and rear["lon"] is not None
        )
        
        # If we have both GPS positions, calculate heading from their relative positions
        if both_gps_valid:
            # ALWAYS recalculate heading directly from positions
            calculated_heading = self.calculate_bearing(
                rear["lat"], rear["lon"],
                front["lat"], front["lon"]
            )
            
            # Don't use the heading stored in GPS data, use our calculated one
            # Store this for future reference
            self._last_calculated_heading = calculated_heading
            self._last_heading_time = time.time()
            
            # Log that we're using freshly calculated heading
            logger.debug(f"Using fresh calculated heading: {calculated_heading:.1f}°")
            return front["lat"], front["lon"], calculated_heading, front["speed"]
        
        # If we don't have both GPS positions, check for alternatives
        # First try NMEA heading from front GPS
        if front["lat"] is not None and front["lon"] is not None and front["heading"] is not None:
            logger.debug(f"Using NMEA heading from front GPS: {front['heading']:.1f}°")
            return front["lat"], front["lon"], front["heading"], front["speed"]
        
        # Check if we have a recent calculated heading (within 5 seconds)
        if hasattr(self, '_last_calculated_heading') and time.time() - self._last_heading_time < 5.0:
            logger.debug(f"Using recent calculated heading: {self._last_calculated_heading:.1f}°")
            return front["lat"], front["lon"], self._last_calculated_heading, front["speed"]
        
        # We don't have sufficient data to provide a heading
        logger.warning("Cannot calculate heading: Need both GPS positions or NMEA heading")
        return front["lat"], front["lon"], None, front["speed"]
    
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

    def _start_gps_monitor_thread(self):
        """Start a thread to monitor GPS connectivity and quality"""
        self.monitor_thread = threading.Thread(
            target=self._gps_monitor_function,
            daemon=True
        )
        self.monitor_thread.start()
    
    def _gps_monitor_function(self):
        """Monitor GPS connectivity and quality"""
        last_report = 0
        last_devices_count = 0
        
        while self.running:
            try:
                now = time.time()
                
                # Generate a report every 30 seconds or if issues detected
                front_ok = self.gps_data["front"]["lat"] is not None
                rear_ok = self.gps_data["rear"]["lat"] is not None
                
                if (now - last_report > 30) or (not front_ok or not rear_ok):
                    self._report_gps_status()
                    last_report = now
                    
                    # If missing GPS data for too long, try to rediscover devices
                    if not front_ok or not rear_ok:
                        logger.warning("GPS data missing, attempting to reconnect...")
                        self._reconnect_gps_devices()
                
                time.sleep(5)
                
            except Exception as e:
                logger.error(f"Error in GPS monitor thread: {e}")
                time.sleep(10)  # Longer delay on error

    def _report_gps_status(self):
        """Generate a detailed report of GPS status with screen clearing"""
        
        front = self.gps_data["front"]
        rear = self.gps_data["rear"]
        
        # Check how long since the last update
        now = time.time()
        front_age = now - front.get("last_update", 0)
        rear_age = now - rear.get("last_update", 0)
        
        # Build the report
        logger.info("═══════ GPS STATUS REPORT ═══════")
        
        # Front GPS status
        if front["lat"] is not None:
            fix_type = "RTK Fixed" if front["fix_quality"] >= 4 else (
                       "RTK Float" if front["fix_quality"] == 5 else (
                       "DGPS" if front["fix_quality"] == 2 else "3D Fix"))
            
            logger.info(f"FRONT GPS: ✅ OK - {fix_type} ({front['satellites']} sats, {front_age:.1f}s ago)")
            logger.info(f"           Position: {front['lat']:.7f}, {front['lon']:.7f}")
            logger.info(f"           NMEA Heading: {front['heading']}")
        else:
            logger.warning(f"FRONT GPS: ❌ NO DATA - Last seen: {front_age:.1f}s ago")
        
        # Rear GPS status
        if rear["lat"] is not None:
            fix_type = "RTK Fixed" if rear["fix_quality"] >= 4 else (
                       "RTK Float" if rear["fix_quality"] == 5 else (
                       "DGPS" if rear["fix_quality"] == 2 else "3D Fix"))
            
            logger.info(f"REAR GPS:  ✅ OK - {fix_type} ({rear['satellites']} sats, {rear_age:.1f}s ago)")
            logger.info(f"           Position: {rear['lat']:.7f}, {rear['lon']:.7f}")
        else:
            logger.warning(f"REAR GPS:  ❌ NO DATA - Last seen: {rear_age:.1f}s ago")
        
        # Check for dual-GPS availability
        if front["lat"] is not None and rear["lat"] is not None:
            distance = self.calculate_distance((rear["lat"], rear["lon"]), (front["lat"], front["lon"]))
            dual_heading = self.calculate_bearing(rear["lat"], rear["lon"], front["lat"], front["lon"])
            logger.info(f"DUAL GPS:  ✅ OK - Distance: {distance:.2f}m, Calculated Heading: {dual_heading:.1f}°")
        else:
            logger.warning("DUAL GPS:  ❌ NOT AVAILABLE - Need both GPS units for accurate heading")
        
        logger.info("═════════════════════════════════")

    def _rediscover_gps_devices(self):
        """Try to rediscover GPS devices and reassign roles"""
        if not hasattr(self, 'gpsd_client'):
            return
        
        # Send a fresh POLL command to ensure we have current data
        self.gpsd_client.send_poll()
        time.sleep(1)  # Give GPSD time to respond
        
        # Process any pending reports
        self.gpsd_client.process_reports()
        
        # Get all available devices
        devices = list(self.gpsd_client.devices.keys())
        if not devices:
            logger.warning("No GPS devices found during rediscovery")
            return
        
        # If we have a front GPS but no rear GPS, and there's another device available,
        # try to assign it as the rear GPS
        front_device = self.gps_data["front"].get("device_path")
        if front_device and not self.gps_data["rear"].get("device_path"):
            for device in devices:
                if device != front_device:
                    logger.info(f"Reassigning {device} as rear GPS")
                    rear_data = self.gpsd_client.devices[device]
                    
                    # Update rear GPS data
                    self.gps_data["rear"]["device_path"] = device
                    self.gps_data["rear"]["lat"] = rear_data.get('lat')
                    self.gps_data["rear"]["lon"] = rear_data.get('lon')
                    self.gps_data["rear"]["heading"] = rear_data.get('track')
                    self.gps_data["rear"]["speed"] = rear_data.get('speed', 0.0)
                    self.gps_data["rear"]["fix_quality"] = rear_data.get('fix_quality', 0)
                    self.gps_data["rear"]["last_update"] = rear_data.get('last_update', 0)
                    self.gps_data["rear"]["satellites"] = rear_data.get('satellites_used', 0)
                    self.gps_data["rear"]["hdop"] = rear_data.get('hdop', 99.9)
                    return

    def _reconnect_gps_devices(self):
        """Try to reconnect to GPS devices if they're disconnected"""
        if self.simulation_mode:
            return
            
        try:
            # For serial implementation
            if hasattr(self, 'front_serial') and not self.front_serial.is_open:
                logger.info("Attempting to reopen front GPS serial port...")
                self.front_serial.open()
                
            if hasattr(self, 'rear_serial') and not self.rear_serial.is_open:
                logger.info("Attempting to reopen rear GPS serial port...")
                self.rear_serial.open()
                
            # Reset messages_per_sec counters if we're not receiving data
            now = time.time()
            if now - self.gps_data["front"]["last_update"] > 10:
                self.gps_data["front"]["messages_per_sec"] = 0
                
            if now - self.gps_data["rear"]["last_update"] > 10:
                self.gps_data["rear"]["messages_per_sec"] = 0
                
        except Exception as e:
            logger.error(f"Error reconnecting to GPS devices: {e}")

    def _recalculate_heading(self):
        """Force recalculation of heading from current GPS data"""
        try:
            # Get current GPS positions
            front_lat = self.gps_data["front"]["lat"]
            front_lon = self.gps_data["front"]["lon"]
            rear_lat = self.gps_data["rear"]["lat"]
            rear_lon = self.gps_data["rear"]["lon"]
            
            # Only calculate if we have valid data
            if None not in [front_lat, front_lon, rear_lat, rear_lon]:
                # Calculate bearing from rear to front GPS
                heading = self.calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
                
                # Force store the calculated heading directly, overriding any NMEA value
                self.gps_data["front"]["heading"] = heading
                
                # Store this calculated heading for future use
                self._last_calculated_heading = heading
                self._last_heading_time = time.time()
                
                logger.info(f"Heading recalculated: {heading:.1f}°")
                return heading
            else:
                logger.debug("Cannot recalculate heading - missing GPS positions")
                return None
        except Exception as e:
            logger.error(f"Error recalculating heading: {e}")
            return None

    def _calculate_dual_gps_heading(self):
        """Calculate heading between rear and front GPS, used for debugging"""
        try:
            front_lat = self.gps_data["front"]["lat"]
            front_lon = self.gps_data["front"]["lon"]
            rear_lat = self.gps_data["rear"]["lat"]
            rear_lon = self.gps_data["rear"]["lon"]
            
            if None not in [front_lat, front_lon, rear_lat, rear_lon]:
                return self.calculate_bearing(rear_lat, rear_lon, front_lat, front_lon)
            return None
        except:
            return None

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