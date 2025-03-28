#!/usr/bin/env python3

import subprocess
import sys
import time
import os
import signal
import atexit

def cleanup_processes(processes):
    """Clean up all spawned processes"""
    for proc in processes:
        if proc and proc.poll() is None:
            print(f"Terminating process: {proc.args}")
            proc.terminate()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()

def setup_ssh_bridge(serial_port='/dev/ttyUSB0', local_port=2222):
    """Set up an SSH bridge over a LoRa radio connection"""
    processes = []
    
    # Make sure the port exists
    if not os.path.exists(serial_port):
        print(f"ERROR: Serial port {serial_port} does not exist!")
        return

    # Clean up virtual modem if it exists
    if os.path.exists("/tmp/vmodem"):
        try:
            os.remove("/tmp/vmodem")
            print("Removed existing virtual modem")
        except Exception as e:
            print(f"Error removing virtual modem: {e}")
    
    print(f"Setting up SSH bridge: {serial_port} -> localhost:{local_port}")
    
    # Create a PTY pair for socat to use
    socat_cmd = [
        "socat",
        f"PTY,link=/tmp/vmodem,raw,echo=0,waitslave",
        f"FILE:{serial_port},b57600,raw"
    ]
    
    try:
        serial_proc = subprocess.Popen(
            socat_cmd, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE
        )
        processes.append(serial_proc)
        print(f"Started serial bridge: {' '.join(socat_cmd)}")
        
        # Give socat time to create the virtual modem
        time.sleep(1)
        
        if not os.path.exists("/tmp/vmodem"):
            stderr = serial_proc.stderr.read().decode('utf-8')
            print(f"ERROR: Virtual modem not created. socat error: {stderr}")
            cleanup_processes(processes)
            return
        
        # Forward SSH over the virtual modem
        ssh_cmd = [
            "socat",
            f"TCP-LISTEN:{local_port},reuseaddr,fork",
            "FILE:/tmp/vmodem,raw"
        ]
        
        ssh_proc = subprocess.Popen(
            ssh_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        processes.append(ssh_proc)
        print(f"Started SSH tunnel: {' '.join(ssh_cmd)}")
        
        # Register cleanup function
        atexit.register(cleanup_processes, processes)
        
        print(f"\nSSH bridge established!")
        print(f"Connect with: ssh -p {local_port} username@localhost")
        print("Press Ctrl+C to exit")
        
        # Keep the bridge running
        while True:
            if serial_proc.poll() is not None:
                stderr = serial_proc.stderr.read().decode('utf-8')
                print(f"Serial bridge died with code {serial_proc.returncode}: {stderr}")
                break
                
            if ssh_proc.poll() is not None:
                stderr = ssh_proc.stderr.read().decode('utf-8')
                print(f"SSH tunnel died with code {ssh_proc.returncode}: {stderr}")
                break
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down SSH bridge...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cleanup_processes(processes)
        if os.path.exists("/tmp/vmodem"):
            try:
                os.remove("/tmp/vmodem")
                print("Removed virtual modem")
            except Exception as e:
                print(f"Error removing virtual modem: {e}")

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
    setup_ssh_bridge(port)