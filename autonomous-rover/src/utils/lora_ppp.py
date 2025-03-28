#!/usr/bin/env python3

import subprocess
import sys
import time
import os
import signal
import argparse
import threading

def run_command(cmd, description=None):
    """Run a command and return process"""
    if description:
        print(f"Starting {description}...")
        print(f"Command: {' '.join(cmd)}")
    
    try:
        process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        return process
    except Exception as e:
        print(f"Error starting {description}: {e}")
        return None

def setup_server_ppp(serial_port):
    """Set up PPP server (on control computer)"""
    # Create PPP configuration
    ppp_config = f"""
# LoRa PPP configuration
{serial_port}
57600
lock
noauth
local
nodetach
persist
nocrtscts
xonxoff
nodefaultroute
proxyarp
usepeerdns
asyncmap 0
lcp-echo-failure 5
lcp-echo-interval 10
"""
    
    # Write config to file
    config_path = "/tmp/lora-ppp"
    with open(config_path, "w") as f:
        f.write(ppp_config)
    
    # Start PPP
    ppp_cmd = [
        "sudo",
        "pppd",
        "file", config_path,
        "192.168.42.1:192.168.42.2",
        "passive"
    ]
    
    print("Starting PPP server...")
    print(f"PPP will assign 192.168.42.1 to this computer and 192.168.42.2 to the rover")
    print(f"After connection is established, you can SSH to the rover with:")
    print(f"  ssh pi@192.168.42.2")
    
    return run_command(ppp_cmd, "PPP server")

def setup_client_ppp(serial_port):
    """Set up PPP client (on rover)"""
    # Create PPP configuration
    ppp_config = f"""
# LoRa PPP configuration
{serial_port}
57600
lock
noauth
local
nodetach
persist
nocrtscts
xonxoff
defaultroute
proxyarp
asyncmap 0
lcp-echo-failure 5
lcp-echo-interval 10
"""
    
    # Write config to file
    config_path = "/tmp/lora-ppp"
    with open(config_path, "w") as f:
        f.write(ppp_config)
    
    # Start PPP
    ppp_cmd = [
        "sudo",
        "pppd",
        "file", config_path
    ]
    
    print("Starting PPP client...")
    print("Once connected, this system will be accessible at 192.168.42.2")
    
    return run_command(ppp_cmd, "PPP client")

def monitor_process(process, name):
    """Monitor a process and print its output"""
    while process.poll() is None:
        output = process.stdout.readline().decode('utf-8', errors='ignore').strip()
        if output:
            print(f"[{name}] {output}")
        error = process.stderr.readline().decode('utf-8', errors='ignore').strip()
        if error:
            print(f"[{name} ERROR] {error}")
        time.sleep(0.1)
    
    return_code = process.poll()
    if return_code != 0:
        stderr = process.stderr.read().decode('utf-8', errors='ignore')
        if stderr:
            print(f"[{name}] Process exited with code {return_code}: {stderr}")
        else:
            print(f"[{name}] Process exited with code {return_code}")

def main():
    parser = argparse.ArgumentParser(description='LoRa PPP Setup')
    parser.add_argument('--server', action='store_true', help='Run in server mode (on control computer)')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port to use')
    
    args = parser.parse_args()
    
    # Check for root permissions for pppd
    if os.geteuid() != 0:
        print("This script requires root privileges to run PPP.")
        print("Please run with sudo.")
        return
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nInterrupted by user, shutting down...")
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # Run in appropriate mode
    if args.server:
        process = setup_server_ppp(args.port)
    else:
        process = setup_client_ppp(args.port)
    
    if process:
        print("PPP started successfully. Press Ctrl+C to exit.")
        monitor_process(process, "pppd")
    else:
        print("Failed to start PPP.")

if __name__ == "__main__":
    main()