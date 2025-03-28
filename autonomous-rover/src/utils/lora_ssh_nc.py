#!/usr/bin/env python3

import subprocess
import time
import argparse
import sys
import os
import signal
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

def cleanup(processes):
    """Clean up processes"""
    for name, process in processes.items():
        if process and process.poll() is None:
            print(f"Terminating {name}...")
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()

def server_mode(serial_port, ssh_port):
    """Run in server mode - accept SSH connections and forward to serial"""
    processes = {}
    
    # Create virtual serial port pair with socat
    socat_cmd = [
        "socat",
        f"PTY,link=/tmp/vmodem,raw,echo=0,waitslave",
        f"FILE:{serial_port},b57600,raw"
    ]
    
    processes["socat"] = run_command(socat_cmd, "serial port bridge")
    
    # Give socat time to create the virtual modem
    time.sleep(1)
    
    if not os.path.exists("/tmp/vmodem"):
        print("ERROR: Failed to create virtual modem")
        cleanup(processes)
        return
    
    # Start netcat to listen on SSH port and forward to virtual modem
    netcat_cmd = [
        "nc",
        "-l",
        "-p", str(ssh_port),
        "-e", f"cat /tmp/vmodem"
    ]
    
    processes["netcat"] = run_command(netcat_cmd, "SSH listener")
    
    # Monitor processes
    socat_thread = threading.Thread(target=monitor_process, args=(processes["socat"], "socat"))
    socat_thread.daemon = True
    socat_thread.start()
    
    netcat_thread = threading.Thread(target=monitor_process, args=(processes["netcat"], "netcat"))
    netcat_thread.daemon = True
    netcat_thread.start()
    
    print(f"Server ready! SSH connections on port {ssh_port} will be forwarded to {serial_port}")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            time.sleep(1)
            # Check if processes are still running
            for name, process in processes.items():
                if process.poll() is not None:
                    print(f"{name} process exited")
                    return
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cleanup(processes)
        if os.path.exists("/tmp/vmodem"):
            try:
                os.remove("/tmp/vmodem")
            except:
                pass

def client_mode(serial_port):
    """Run in client mode - forward from serial to SSH server"""
    processes = {}
    
    # Create virtual serial port pair with socat
    socat_cmd = [
        "socat",
        f"PTY,link=/tmp/vmodem,raw,echo=0,waitslave",
        f"FILE:{serial_port},b57600,raw"
    ]
    
    processes["socat"] = run_command(socat_cmd, "serial port bridge")
    
    # Give socat time to create the virtual modem
    time.sleep(1)
    
    if not os.path.exists("/tmp/vmodem"):
        print("ERROR: Failed to create virtual modem")
        cleanup(processes)
        return
    
    # Start netcat to connect to SSH server and forward to virtual modem
    netcat_cmd = [
        "cat",
        "/tmp/vmodem"
    ]
    
    processes["netcat"] = run_command(netcat_cmd, "SSH connector")
    
    # Monitor processes
    socat_thread = threading.Thread(target=monitor_process, args=(processes["socat"], "socat"))
    socat_thread.daemon = True
    socat_thread.start()
    
    netcat_thread = threading.Thread(target=monitor_process, args=(processes["netcat"], "netcat"))
    netcat_thread.daemon = True
    netcat_thread.start()
    
    print(f"Client ready! Serial data from {serial_port} will be forwarded to SSH")
    print("Press Ctrl+C to exit")
    
    try:
        while True:
            time.sleep(1)
            # Check if processes are still running
            for name, process in processes.items():
                if process.poll() is not None:
                    print(f"{name} process exited")
                    return
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        cleanup(processes)
        if os.path.exists("/tmp/vmodem"):
            try:
                os.remove("/tmp/vmodem")
            except:
                pass

def main():
    parser = argparse.ArgumentParser(description='LoRa SSH Relay using netcat')
    parser.add_argument('--server', action='store_true', help='Run in server mode')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port to use')
    parser.add_argument('--ssh-port', type=int, default=2222, help='SSH port to listen on (server mode)')
    
    args = parser.parse_args()
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nInterrupted, shutting down...")
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    if args.server:
        server_mode(args.port, args.ssh_port)
    else:
        client_mode(args.port)

if __name__ == "__main__":
    main()