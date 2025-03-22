#!/usr/bin/env python3
import socket
import json
import time
import os
import subprocess

def test_gpsd():
    print("Attempting to connect to GPSD...")
    
    try:
        # Connect to GPSD
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("localhost", 2947))
        print("Connected to GPSD")
        
        # Enable watching with comprehensive options
        watch_cmd = '?WATCH={"enable":true,"json":true,"nmea":true,"raw":1,"scaled":true};\n'
        sock.send(watch_cmd.encode())
        print("Sent WATCH command")
        
        # Request device list
        sock.send(b'?DEVICES;\n')
        print("Sent DEVICES command")
        
        # Request poll for immediate data
        sock.send(b'?POLL;\n')
        print("Sent POLL command")
        
        # Receive data for 15 seconds (more time to get satellite info)
        print("\nWaiting for GPSD data (15 seconds)...")
        start_time = time.time()
        devices = {}
        buffer = ""
        
        # Track statistics
        report_counts = {
            'DEVICES': 0,
            'TPV': 0,
            'SKY': 0,
            'TOTAL': 0
        }
        
        while time.time() - start_time < 15:
            try:
                sock.settimeout(1.0)
                data = sock.recv(4096).decode('utf-8', errors='replace')
                
                if data:
                    print(f"Received {len(data)} bytes")
                    buffer += data
                    
                    # Process complete JSON objects
                    while True:
                        # Find start of a JSON object
                        start_idx = buffer.find('{')
                        if start_idx == -1:
                            buffer = ""  # No JSON start found
                            break
                            
                        # Find the end of the JSON object by matching braces
                        end_idx = -1
                        depth = 0
                        for i in range(start_idx, len(buffer)):
                            if buffer[i] == '{':
                                depth += 1
                            elif buffer[i] == '}':
                                depth -= 1
                                if depth == 0:
                                    end_idx = i
                                    break
                                    
                        if end_idx == -1:
                            break  # No complete JSON object
                            
                        # Extract complete JSON object
                        json_str = buffer[start_idx:end_idx+1]
                        
                        # Find where to continue parsing (after semicolon if present)
                        semi_idx = buffer.find(';', end_idx)
                        if semi_idx != -1:
                            buffer = buffer[semi_idx+1:]
                        else:
                            buffer = buffer[end_idx+1:]
                            
                        # Parse the JSON
                        try:
                            report = json.loads(json_str)
                            report_class = report.get('class', 'unknown')
                            report_counts['TOTAL'] += 1
                            
                            if report_class in report_counts:
                                report_counts[report_class] += 1
                                
                            print(f"Parsed {report_class} report")
                            
                            if report_class == 'DEVICES':
                                devices_list = report.get('devices', [])
                                print(f"📱 Found {len(devices_list)} GPS devices:")
                                
                                for device in devices_list:
                                    path = device.get('path', 'unknown')
                                    driver = device.get('driver', 'unknown')
                                    
                                    # Extract model info if available
                                    model = "unknown"
                                    subtype1 = device.get('subtype1', '')
                                    if subtype1 and "MOD=" in subtype1:
                                        import re
                                        model_match = re.search(r'MOD=([^,]+)', subtype1)
                                        if model_match:
                                            model = model_match.group(1)
                                    
                                    print(f"  - {path} ({driver}, {model})")
                                    
                                    # Store device info
                                    devices[path] = {
                                        'driver': driver,
                                        'model': model,
                                        'subtype': device.get('subtype', ''),
                                        'subtype1': subtype1
                                    }
                                
                            elif report_class == 'TPV' and 'device' in report:
                                device_path = report.get('device')
                                lat = report.get('lat')
                                lon = report.get('lon')
                                mode = report.get('mode', 0)
                                
                                # Display position info if available
                                if lat is not None and lon is not None:
                                    print(f"📍 Position from {device_path}: {lat:.7f}, {lon:.7f}")
                                    print(f"   Fix Mode: {mode}, Time: {report.get('time', 'unknown')}")
                                    
                                    # Store in devices dict
                                    if device_path not in devices:
                                        devices[device_path] = {}
                                    devices[device_path].update({
                                        'lat': lat,
                                        'lon': lon, 
                                        'mode': mode,
                                        'has_fix': mode >= 2
                                    })
                                
                            elif report_class == 'SKY' and 'device' in report:
                                device_path = report.get('device')
                                satellites = report.get('satellites', [])
                                used_count = sum(1 for s in satellites if s.get('used'))
                                total_count = len(satellites)
                                
                                print(f"🛰️ Satellites from {device_path}: {used_count}/{total_count} in use")
                                
                                # Store in devices dict
                                if device_path not in devices:
                                    devices[device_path] = {}
                                devices[device_path].update({
                                    'satellites_used': used_count,
                                    'satellites_visible': total_count,
                                    'hdop': report.get('hdop', 0.0),
                                    'vdop': report.get('vdop', 0.0)
                                })
                                
                        except json.JSONDecodeError as e:
                            # This might just be incomplete data
                            pass
                        
            except socket.timeout:
                # Timeout is normal, send another poll
                sock.send(b'?POLL;\n')
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
                break
        
        # Final report
        print("\n=== GPSD Test Results ===")
        print(f"Reports processed: {report_counts['TOTAL']} total")
        print(f"  DEVICES reports: {report_counts['DEVICES']}")  
        print(f"  TPV reports: {report_counts['TPV']}")
        print(f"  SKY reports: {report_counts['SKY']}")
        
        if devices:
            print(f"\nDetected {len(devices)} GPS device(s):")
            for path, info in devices.items():
                satellites = info.get('satellites_used', 'unknown')
                fix = "YES" if info.get('has_fix') else "NO"
                model = info.get('model', 'unknown')
                
                print(f"  - {path} ({model})")
                print(f"    Satellites: {satellites}, Has Fix: {fix}")
                if 'lat' in info and 'lon' in info:
                    print(f"    Position: {info['lat']:.7f}, {info['lon']:.7f}")
        else:
            print("NO GPS DEVICES DETECTED!")
            print("\nPossible issues:")
            print("1. GPSD service is not properly configured")
            print("2. GPS devices are not physically connected")
            print("3. GPS devices need different drivers")
            print("4. USB permissions issues")
            
        return devices
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return {}
    finally:
        try:
            sock.close()
        except:
            pass

def check_hardware():
    """Check hardware connections directly"""
    print("\n=== HARDWARE CHECK ===")
    
    # Check USB devices
    print("\nUSB Devices:")
    try:
        lsusb = subprocess.run(['lsusb'], capture_output=True, text=True)
        for line in lsusb.stdout.splitlines():
            # Highlight GPS-related devices
            if any(term in line.lower() for term in ['blox', 'gps', '1546']):
                print(f"🛰️ {line} (Possible GPS)")
            else:
                print(f"  {line}")
    except Exception as e:
        print(f"Error checking USB devices: {e}")
    
    # Check serial ports
    print("\nSerial Ports:")
    try:
        for port in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if os.path.exists(port):
                print(f"✓ {port} exists")
                
                # Get device info
                try:
                    cmd = f"udevadm info -a -n {port} | grep -E 'idVendor|idProduct|manufacturer|product' | head -4"
                    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
                    if result.stdout:
                        for line in result.stdout.splitlines():
                            print(f"  {line.strip()}")
                except:
                    pass
            else:
                print(f"✗ {port} does not exist")
    except Exception as e:
        print(f"Error checking serial ports: {e}")

def check_gpsd_service():
    """Check GPSD service status and configuration"""
    print("\n=== GPSD SERVICE CHECK ===")
    
    # Check service status
    print("\nGPSD Service Status:")
    try:
        status = subprocess.run(['systemctl', 'status', 'gpsd'], capture_output=True, text=True)
        if 'Active: active' in status.stdout:
            print("✓ GPSD service is active")
        else:
            print("✗ GPSD service is not active")
        
        for line in status.stdout.splitlines()[:10]:
            print(f"  {line}")
    except:
        print("  Could not check GPSD service status")
    
    # Check GPSD processes
    print("\nGPSD Processes:")
    try:
        ps = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        for line in ps.stdout.splitlines():
            if 'gpsd' in line:
                print(f"  {line}")
    except:
        print("  Could not check GPSD processes")
    
    # Check configuration
    print("\nGPSD Configuration File:")
    try:
        with open('/etc/default/gpsd', 'r') as f:
            config = f.read()
            print(config)
    except:
        print("  Could not read GPSD configuration file")

def restart_gpsd():
    """Restart GPSD service with explicit device configuration"""
    print("\n=== RESTARTING GPSD ===")
    
    try:
        # Stop service
        print("Stopping GPSD service...")
        subprocess.run(['sudo', 'systemctl', 'stop', 'gpsd'], check=True)
        time.sleep(1)
        
        # Find all possible GPS devices
        devices = []
        for port in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            if os.path.exists(port):
                devices.append(port)
        
        if not devices:
            print("No serial devices found, cannot restart GPSD")
            return
        
        # Start GPSD with discovered devices
        print(f"Starting GPSD with devices: {' '.join(devices)}")
        cmd = ['sudo', 'gpsd', '-N', '-n'] + devices
        subprocess.run(cmd, check=True)
        time.sleep(2)
        
        print("GPSD restarted with available devices")
    except Exception as e:
        print(f"Failed to restart GPSD: {e}")

if __name__ == "__main__":
    # Test GPSD connection
    devices = test_gpsd()
    
    # If no devices found, run diagnostics
    if not devices:
        print("\nNo GPS devices detected. Running diagnostics...")
        check_hardware()
        check_gpsd_service()
        
        # Ask user if they want to restart GPSD
        print("\nWould you like to restart GPSD with all available devices? (y/n)")
        response = input().strip().lower()
        if response == 'y':
            restart_gpsd()
            time.sleep(2)
            print("\nTesting GPSD again after restart...")
            devices = test_gpsd()