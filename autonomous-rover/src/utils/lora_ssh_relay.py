#!/usr/bin/env python3

import serial
import threading
import socket
import sys
import time
import os
import signal
import traceback
import argparse
import queue
import binascii

# Configuration
LOCAL_SSH_PORT = 2222  # Port to listen for SSH connections
CHUNK_SIZE = 64        # Reduced chunk size for better reliability
SERIAL_SPEED = 57600   # LoRa baud rate
DEBUG = True           # Enable debug output

def debug_print(msg):
    """Print debug messages if DEBUG is enabled"""
    if DEBUG:
        print(f"DEBUG: {msg}")

def hex_dump(data, prefix=""):
    """Create a hex dump of binary data for debugging"""
    if DEBUG and data:
        hex_str = binascii.hexlify(data).decode('ascii')
        debug_print(f"{prefix}HEX: {hex_str[:100]}{'...' if len(hex_str) > 100 else ''}")

class LoRaSSHRelay:
    def __init__(self, serial_port='/dev/ttyUSB0', is_server=False):
        self.serial_port = serial_port
        self.is_server = is_server
        self.ser = None
        self.server_socket = None
        self.client = None
        self.running = False
        self.packet_counter = 0
        self.tx_queue = queue.Queue()  # Queue for data to be sent over serial
    
    def start(self):
        """Start the relay"""
        print(f"Starting LoRa SSH relay on {self.serial_port}")
        print(f"Mode: {'SERVER' if self.is_server else 'CLIENT'}")
        
        try:
            # Check if serial port exists
            if not os.path.exists(self.serial_port):
                print(f"ERROR: Serial port {self.serial_port} does not exist!")
                return
                
            # Open serial connection
            debug_print(f"Opening serial port {self.serial_port} at {SERIAL_SPEED} baud")
            try:
                self.ser = serial.Serial(self.serial_port, SERIAL_SPEED, timeout=0.1)
            except serial.SerialException as e:
                print(f"Failed to open serial port: {e}")
                return
                
            debug_print("Serial port opened successfully")
            time.sleep(1)  # Give serial port time to initialize
            
            # Start serial reader thread
            self.running = True
            
            # Start serial writer thread
            writer_thread = threading.Thread(target=self.serial_writer, name="SerialWriter")
            writer_thread.daemon = True
            writer_thread.start()
            
            # Start serial reader thread
            reader_thread = threading.Thread(target=self.read_from_serial, name="SerialReader")
            reader_thread.daemon = True
            reader_thread.start()
            
            # Send test message
            self.queue_for_serial(b"Relay starting up", is_ssh=False)
            
            if self.is_server:
                # Server mode: Listen for SSH connections and forward to LoRa
                self.run_server()
            else:
                # Client mode: Forward data from LoRa to SSH server
                self.run_client()
                
        except KeyboardInterrupt:
            print("\nShutting down due to keyboard interrupt...")
        except Exception as e:
            print(f"Error during startup: {e}")
            traceback.print_exc()
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up resources...")
        self.running = False
        
        if self.ser and self.ser.is_open:
            debug_print("Closing serial port")
            try:
                self.ser.close()
            except:
                pass
                
        if self.server_socket:
            debug_print("Closing server socket")
            try:
                self.server_socket.close()
            except:
                pass
                
        if self.client:
            debug_print("Closing client socket")
            try:
                self.client.close()
            except:
                pass
                
        if hasattr(self, 'ssh_socket') and self.ssh_socket:
            debug_print("Closing SSH socket")
            try:
                self.ssh_socket.close()
            except:
                pass
                
        print("Relay shutdown complete")
    
    def queue_for_serial(self, data, is_ssh=True):
        """Queue data to be sent over serial"""
        if not data:
            return
        
        # Put data in queue as tuple (data, is_ssh)
        self.tx_queue.put((data, is_ssh))
    
    def serial_writer(self):
        """Thread that handles sending data from queue to serial port"""
        debug_print("Serial writer thread started")
        sequence_number = 0
        
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.5)
                    continue
                
                try:
                    # Get data from queue with timeout
                    data, is_ssh = self.tx_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                
                # Split data into smaller chunks
                debug_print(f"Sending {len(data)} bytes over serial (is_ssh={is_ssh})")
                if is_ssh:
                    hex_dump(data, "TX: ")
                
                for i in range(0, len(data), CHUNK_SIZE):
                    chunk = data[i:i+CHUNK_SIZE]
                    
                    # Add sequence number for reassembly
                    if is_ssh:
                        # Format: SSH:[sequence number]:[total chunks]:[chunk number]:[data]
                        total_chunks = (len(data) + CHUNK_SIZE - 1) // CHUNK_SIZE
                        chunk_num = i // CHUNK_SIZE + 1
                        header = f"SSH:{sequence_number:04d}:{total_chunks:02d}:{chunk_num:02d}:".encode()
                        packet = header + chunk
                        
                        if chunk_num == total_chunks:
                            sequence_number = (sequence_number + 1) % 10000
                    else:
                        packet = chunk + b'\n'  # Add newline for regular messages
                        
                    try:
                        # Send with verification
                        bytes_written = self.ser.write(packet)
                        self.ser.flush()
                        
                        # Wait longer between SSH packets - critical for LoRa
                        if is_ssh:
                            time.sleep(0.5)  # Much longer delay for SSH packets
                        else:
                            time.sleep(0.1)
                            
                        self.packet_counter += 1
                        if self.packet_counter % 5 == 0:
                            print(f"Sent {self.packet_counter} packets", end='\r', flush=True)
                    except Exception as e:
                        print(f"Error sending to serial: {e}")
                        traceback.print_exc()
                        break
                
                # Additional delay after sending a complete message
                if is_ssh and len(data) > CHUNK_SIZE:
                    time.sleep(1.0)  # Give more time for larger messages
                    
                self.tx_queue.task_done()
                    
            except Exception as e:
                print(f"Error in serial writer: {e}")
                traceback.print_exc()
                time.sleep(1)
        
        debug_print("Serial writer thread exiting")
    
    def read_from_serial(self):
        """Read data from serial port and process it"""
        debug_print("Serial reader thread started")
        
        # Buffer for reassembling SSH packets
        ssh_buffers = {}  # {sequence_number: [chunks]}
        buffer = b''  # buffer for incomplete data
        
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    debug_print("Serial port not open in reader thread")
                    time.sleep(1)
                    continue
                    
                if self.ser.in_waiting:
                    # Read available data and add to buffer
                    new_data = self.ser.read(self.ser.in_waiting)
                    if new_data:
                        buffer += new_data
                        debug_print(f"Received {len(new_data)} bytes, buffer size: {len(buffer)}")
                        
                        # Process complete SSH packets from buffer
                        while buffer:
                            # Look for packet start
                            if not buffer.startswith(b'SSH:'):
                                # No SSH prefix found, check for regular message
                                newline_pos = buffer.find(b'\n')
                                if newline_pos >= 0:
                                    # Extract regular message up to newline
                                    message = buffer[:newline_pos].decode('utf-8', errors='replace').strip()
                                    buffer = buffer[newline_pos + 1:]
                                    print(f"\nRX: {message}")
                                else:
                                    # Wait for more data
                                    break
                            else:
                                # Try to parse SSH header
                                try:
                                    # Find header parts: SSH:NNNN:TT:CC:
                                    header_parts = buffer.split(b':', 4)
                                    if len(header_parts) < 5:
                                        # Incomplete header, wait for more data
                                        break
                                        
                                    seq_str = header_parts[1].decode('ascii')
                                    sequence = int(seq_str)
                                    
                                    total_chunks = int(header_parts[2].decode('ascii'))
                                    chunk_num = int(header_parts[3].decode('ascii'))
                                    
                                    # Calculate header length (to extract payload)
                                    header_len = len(header_parts[0]) + len(header_parts[1]) + len(header_parts[2]) + len(header_parts[3]) + 4
                                    
                                    # Check if we have enough data for the payload
                                    # Assume payload size is CHUNK_SIZE or less
                                    if len(buffer) < header_len + CHUNK_SIZE:
                                        # Wait for more data
                                        break
                                    
                                    # Extract payload - get up to CHUNK_SIZE bytes after header
                                    payload = buffer[header_len:header_len + CHUNK_SIZE]
                                    
                                    # Debug the packet
                                    debug_print(f"Parsed SSH chunk: seq={sequence}, chunk {chunk_num}/{total_chunks}, size={len(payload)}")
                                    
                                    # Initialize buffer for this sequence if needed
                                    if sequence not in ssh_buffers:
                                        ssh_buffers[sequence] = [None] * total_chunks
                                    
                                    # Store chunk
                                    ssh_buffers[sequence][chunk_num-1] = payload
                                    
                                    # Remove processed data from buffer
                                    buffer = buffer[header_len + len(payload):]
                                    
                                    # Check if we have all chunks for this sequence
                                    if None not in ssh_buffers[sequence]:
                                        # Reassemble and process
                                        complete_payload = b''.join(ssh_buffers[sequence])
                                        hex_dump(complete_payload, "RX SSH: ")
                                        
                                        # Forward to appropriate endpoint
                                        if self.is_server and self.client:
                                            debug_print(f"Forwarding {len(complete_payload)} bytes to SSH client")
                                            try:
                                                self.client.sendall(complete_payload)
                                                debug_print("Data forwarded successfully")
                                            except ConnectionError as e:
                                                debug_print(f"Connection error while forwarding: {e}")
                                                break
                                        elif not self.is_server and hasattr(self, 'ssh_socket') and self.ssh_socket:
                                            debug_print(f"Forwarding {len(complete_payload)} bytes to SSH server")
                                            try:
                                                self.ssh_socket.sendall(complete_payload)
                                                debug_print("Data forwarded successfully")
                                            except ConnectionError as e:
                                                debug_print(f"Connection error while forwarding: {e}")
                                                break
                                        
                                        # Remove from buffer
                                        del ssh_buffers[sequence]
                                except (ValueError, IndexError) as e:
                                    # Error parsing, discard first byte and continue
                                    debug_print(f"Error parsing SSH packet: {e}")
                                    buffer = buffer[1:]
                                except Exception as e:
                                    debug_print(f"Unexpected error processing SSH packet: {e}")
                                    traceback.print_exc()
                                    buffer = buffer[1:]
                    
            except Exception as e:
                print(f"Error in serial reader: {e}")
                traceback.print_exc()
                time.sleep(1)
                
            time.sleep(0.01)  # Process more frequently
        debug_print("Serial reader thread exiting")
    
    def run_server(self):
        """Run in server mode - listen for SSH connections and relay to LoRa"""
        debug_print("Starting server mode")
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            debug_print(f"Binding to port {LOCAL_SSH_PORT}")
            self.server_socket.bind(('0.0.0.0', LOCAL_SSH_PORT))  # Listen on all interfaces
            self.server_socket.listen(1)
            
            print(f"Waiting for SSH connection on port {LOCAL_SSH_PORT}...")
            
            while self.running:
                try:
                    # Set a timeout so we can check if we're still running
                    self.server_socket.settimeout(1.0)
                    try:
                        self.client, addr = self.server_socket.accept()
                        print(f"Connection from {addr}")
                        
                        # Start client reader thread
                        client_thread = threading.Thread(target=self.server_client_handler, name="ClientHandler")
                        client_thread.daemon = True
                        client_thread.start()
                        
                        # Send initial message to remote side
                        self.queue_for_serial(b"SSH connection established", is_ssh=False)
                    except socket.timeout:
                        continue
                except Exception as e:
                    if self.running:
                        print(f"Server error: {e}")
                        traceback.print_exc()
                    time.sleep(1)
        except Exception as e:
            print(f"Fatal server error: {e}")
            traceback.print_exc()
    
    def server_client_handler(self):
        """Handle SSH client connection in server mode"""
        debug_print("Client handler thread started")
        try:
            while self.running and self.client:
                try:
                    # Set a timeout so we can check if we're still running
                    self.client.settimeout(0.5)
                    try:
                        data = self.client.recv(1024)
                        if not data:
                            debug_print("Client closed connection")
                            break
                        debug_print(f"Received {len(data)} bytes from SSH client")
                        self.queue_for_serial(data, is_ssh=True)
                    except socket.timeout:
                        continue
                except Exception as e:
                    if self.running:
                        print(f"Client handler error: {e}")
                        traceback.print_exc()
                    break
        except Exception as e:
            print(f"Fatal client handler error: {e}")
            traceback.print_exc()
        finally:
            if self.client:
                try:
                    self.client.close()
                except:
                    pass
                self.client = None
            print("SSH client disconnected")
    
    def run_client(self):
        """Run in client mode - forward data from LoRa to SSH server"""
        debug_print("Starting client mode")
        print("Client mode: Connecting to SSH server...")
        try:
            self.ssh_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            debug_print("Connecting to SSH server on localhost:22")
            try:
                self.ssh_socket.connect(('localhost', 22))  # Connect to local SSH server
                debug_print("Connected to SSH server")
                print("Connected to SSH server")
                
                # Start SSH reader thread
                ssh_thread = threading.Thread(target=self.client_ssh_handler, name="SSHHandler")
                ssh_thread.daemon = True
                ssh_thread.start()
                
                # Send initial message
                self.queue_for_serial(b"SSH client connected to server", is_ssh=False)
                
                # Keep main thread alive
                while self.running:
                    time.sleep(1)
            except ConnectionRefusedError:
                print("ERROR: Connection to local SSH server refused. Is SSH server running?")
                return
        except Exception as e:
            print(f"Client error: {e}")
            traceback.print_exc()
    
    def client_ssh_handler(self):
        """Handle SSH server responses in client mode"""
        debug_print("SSH handler thread started")
        try:
            while self.running and hasattr(self, 'ssh_socket') and self.ssh_socket:
                try:
                    # Set a timeout so we can check if we're still running
                    self.ssh_socket.settimeout(0.5)
                    try:
                        data = self.ssh_socket.recv(1024)
                        if not data:
                            debug_print("SSH server closed connection")
                            break
                        debug_print(f"Received {len(data)} bytes from SSH server")
                        self.queue_for_serial(data, is_ssh=True)
                    except socket.timeout:
                        continue
                except Exception as e:
                    if self.running:
                        print(f"SSH handler error: {e}")
                        traceback.print_exc()
                    break
        except Exception as e:
            print(f"Fatal SSH handler error: {e}")
            traceback.print_exc()
        finally:
            if hasattr(self, 'ssh_socket') and self.ssh_socket:
                try:
                    self.ssh_socket.close()
                except:
                    pass
            print("Disconnected from SSH server")

if __name__ == "__main__":
    # Parse command line arguments properly
    parser = argparse.ArgumentParser(description='LoRa SSH Relay')
    parser.add_argument('--server', action='store_true', help='Run in server mode')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port to use')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    
    args = parser.parse_args()
    
    # Update global debug flag if specified
    if args.debug:
        DEBUG = True
    
    # Get port and server mode from args
    port = args.port
    is_server = args.server
    
    print(f"Starting with port={port}, server_mode={is_server}")
    
    relay = LoRaSSHRelay(port, is_server)
    
    # Try to handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print("\nInterrupted by user, shutting down...")
        relay.running = False
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    relay.start()