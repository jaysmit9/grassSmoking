#!/usr/bin/env python3
import socket
import time
import json
import re

def test_raw_gpsd():
    print("Testing raw NMEA and GNSS output from GPSD...")
    
    try:
        # Connect to GPSD
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(("localhost", 2947))
        print("Connected to GPSD")
        
        # Request raw output AND JSON position data
        watch_cmd = '?WATCH={"enable":true,"raw":1,"nmea":true,"json":true};\n'
        sock.send(watch_cmd.encode())
        print("Requested raw data and JSON position data")
        
        # Also explicitly request device info
        sock.send(b'?DEVICES;\n')
        time.sleep(0.5)
        
        # Send POLL to get immediate data
        sock.send(b'?POLL;\n')
        
        # Read for 30 seconds (longer to catch position data)
        print("Reading data for 30 seconds (press Ctrl+C to stop)...")
        start = time.time()
        
        # Track important NMEA sentences
        nmea_count = 0
        json_count = 0
        position_count = 0
        
        # Buffer for collecting JSON objects
        json_buffer = ""
        in_json = False
        
        try:
            while time.time() - start < 30:
                try:
                    data = sock.recv(4096).decode('ascii', errors='replace')
                    if not data:
                        continue
                    
                    # Process line by line to handle different data types
                    lines = data.replace('\r', '').split('\n')
                    
                    for line in lines:
                        if not line:
                            continue
                            
                        # Check for JSON data
                        if line.startswith('{'):
                            json_buffer = line
                            in_json = True
                            continue
                            
                        # Continue collecting JSON data
                        if in_json:
                            json_buffer += line
                            if '}' in line:
                                in_json = False
                                try:
                                    # Try to parse complete JSON object
                                    json_obj = json.loads(json_buffer)
                                    json_class = json_obj.get('class', '')
                                    json_count += 1
                                    
                                    # Extract and display useful information
                                    if json_class == 'TPV':
                                        position_count += 1
                                        lat = json_obj.get('lat', 'N/A')
                                        lon = json_obj.get('lon', 'N/A')
                                        device = json_obj.get('device', 'unknown')
                                        print(f"\n🛰️ POSITION: {device} - Lat: {lat}, Lon: {lon}")
                                        print(f"  Mode: {json_obj.get('mode', 'N/A')}, Time: {json_obj.get('time', 'N/A')}")
                                        if 'epx' in json_obj:
                                            print(f"  Precision - EPX: {json_obj.get('epx')}, EPY: {json_obj.get('epy')}")
                                    
                                    elif json_class == 'SKY':
                                        sats = json_obj.get('satellites', [])
                                        used = sum(1 for s in sats if s.get('used'))
                                        device = json_obj.get('device', 'unknown')
                                        print(f"\n🛰️ SATELLITES: {device} - {used}/{len(sats)} satellites used")
                                    
                                    elif json_class == 'DEVICES':
                                        devices = json_obj.get('devices', [])
                                        print(f"\n📱 DEVICES: Found {len(devices)} GPS devices:")
                                        for i, device in enumerate(devices, 1):
                                            path = device.get('path', 'unknown')
                                            driver = device.get('driver', 'unknown')
                                            subtype = device.get('subtype1', '')
                                            
                                            # Extract model info if available
                                            model = "unknown"
                                            if "MOD=" in subtype:
                                                model_match = re.search(r'MOD=([^,]+)', subtype)
                                                if model_match:
                                                    model = model_match.group(1)
                                                    
                                            print(f"  Device {i}: {path}")
                                            print(f"    Driver: {driver}")
                                            print(f"    Model: {model}")
                                    
                                except json.JSONDecodeError:
                                    pass  # Incomplete or invalid JSON
                                
                                json_buffer = ""
                        
                        # NMEA sentences start with $
                        elif line.startswith('$'):
                            nmea_count += 1
                            # Only print useful NMEA sentences (positions, satellite info)
                            if any(x in line for x in ['GGA', 'RMC', 'GLL', 'GSA']):
                                print(f"\n📍 NMEA: {line}")
                        
                        # Binary data (just count but don't display)
                        elif line.startswith('b562'):
                            pass  # Skip binary data display
                
                except socket.timeout:
                    time.sleep(0.1)
                    continue
                
                # Periodically request more data
                if int(time.time() - start) % 5 == 0:
                    sock.send(b'?POLL;\n')
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\nUser stopped data collection")
        
        # Summary
        print("\n=== GPSD Data Collection Summary ===")
        print(f"NMEA sentences received: {nmea_count}")
        print(f"JSON objects received: {json_count}")
        print(f"Position reports received: {position_count}")
        
        # Send a final DEVICES query
        print("\nFinal GPSD device list:")
        sock.send(b'?DEVICES;\n')
        time.sleep(1)
        try:
            final_data = sock.recv(4096).decode('ascii', errors='replace')
            if '{"class":"DEVICES"' in final_data:
                devices_json = final_data[final_data.find('{'): final_data.rfind('}')+1]
                devices = json.loads(devices_json)
                if 'devices' in devices:
                    for i, device in enumerate(devices['devices'], 1):
                        print(f"  Device {i}: {device.get('path')}")
                        print(f"    Driver: {device.get('driver')}")
        except:
            print("  Could not retrieve final device list")
    
    except Exception as e:
        print(f"Connection error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            sock.close()
        except:
            pass

if __name__ == "__main__":
    test_raw_gpsd()