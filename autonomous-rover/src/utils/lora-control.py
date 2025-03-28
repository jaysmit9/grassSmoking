#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/lora_ssh.py

import os
import sys
import time
import signal
import logging
import argparse
import subprocess
import threading
import serial
import socket
import select
import binascii

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('LoRa-SSH')

class LoRaSSHBridge:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=57600, 
                 ssh_port=8022, rover_ip='localhost', rover_ssh_port=22,
                 debug=False, echo_test=False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ssh_port = ssh_port
        self.rover_ip = rover_ip
        self.rover_ssh_port = rover_ssh_port
        self.debug = debug
        self.echo_test = echo_test
        
        self.serial = None
        self.server_socket = None
        self.running = False
        self.client_connections = []
        self.stats = {
            'bytes_sent': 0,
            'bytes_received': 0,
            'packets_sent': 0,
            'packets_received': 0,
            'start_time': 0
        }
        
        if self.debug:
            logger.setLevel(logging.DEBUG)
    
    def open_serial(self):
        """Open the serial connection to the LoRa modem"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                bytesize=8,
                parity='N',
                stopbits=1,
                xonxoff=False,
                rtscts=False
            )
            logger.info(f"Connected to LoRa on {self.serial_port} at {self.baud_rate} baud")
            
            if self.debug:
                # Test if we can read/write to the serial port
                self.serial.write(b'+++TEST+++\n')
                time.sleep(0.5)
                if self.serial.in_waiting:
                    resp = self.serial.read(self.serial.in_waiting)
                    logger.debug(f"Serial test response: {resp} ({binascii.hexlify(resp)})")
                else:
                    logger.debug("No response from serial test message")
                
            return True
        except serial.SerialException as e:
            logger.error(f"Failed to open serial port: {e}")
            return False
    
    def start_server(self):
        """Start the SSH bridging server"""
        if not self.open_serial():
            return False
        
        # If in echo test mode, don't start the server
        if self.echo_test:
            return self.start_echo_test()
            
        try:
            # Create server socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            try:
                # Try binding to all interfaces
                self.server_socket.bind(('0.0.0.0', self.ssh_port))
                
                # Verify binding worked
                local_address = self.server_socket.getsockname()
                logger.debug(f"Socket bound to {local_address[0]}:{local_address[1]}")
                
            except socket.error as e:
                logger.error(f"Failed to bind to port {self.ssh_port}: {e}")
                
                # Check if something else is using this port
                try:
                    import subprocess
                    result = subprocess.run(['netstat', '-tuln'], 
                                          capture_output=True, text=True)
                    logger.debug(f"Current network connections:\n{result.stdout}")
                except Exception as e:
                    logger.debug(f"Failed to check network connections: {e}")
                    
                return False
            
            self.server_socket.listen(5)
            self.server_socket.settimeout(1)  # 1 second timeout for accept()
            
            logger.info(f"SSH bridge listening on port {self.ssh_port}")
            logger.info(f"Connect with: ssh -p {self.ssh_port} username@localhost")
            
            # Verify socket is actually listening
            try:
                netstat_cmd = "netstat -tuln | grep " + str(self.ssh_port)
                result = subprocess.run(netstat_cmd, shell=True, 
                                       capture_output=True, text=True)
                if result.stdout:
                    logger.debug(f"Netstat confirms socket is listening:\n{result.stdout}")
                else:
                    logger.warning(f"Netstat could not confirm socket is listening on port {self.ssh_port}")
            except Exception as e:
                logger.debug(f"Error checking netstat: {e}")
            
            # Start the bridge threads
            self.running = True
            self.stats['start_time'] = time.time()
            
            # Start serial reader thread
            serial_thread = threading.Thread(target=self._serial_reader, name="SerialReader")
            serial_thread.daemon = True
            serial_thread.start()
            logger.debug("Serial reader thread started")
            
            # Start connection acceptor thread
            accept_thread = threading.Thread(target=self._accept_connections, name="ConnectionAcceptor")
            accept_thread.daemon = True
            accept_thread.start()
            logger.debug("Connection acceptor thread started")
            
            return True
            
        except Exception as e:
            logger.error(f"Failed to start server: {e}")
            if self.serial and self.serial.is_open:
                self.serial.close()
            return False
    
    def start_echo_test(self):
        """Start echo test mode to verify LoRa communication"""
        logger.info("Starting echo test mode - press Ctrl+C to exit")
        logger.info("Type messages and they will be sent over the LoRa link")
        logger.info("Received messages will be displayed")
        
        self.running = True
        self.stats['start_time'] = time.time()
        
        # Start background reader thread
        reader_thread = threading.Thread(target=self._echo_reader)
        reader_thread.daemon = True
        reader_thread.start()
        
        # Main input thread
        try:
            while self.running:
                try:
                    msg = input("> ")
                    if msg:
                        # Add newline if not present
                        if not msg.endswith('\n'):
                            msg = msg + '\n'
                        
                        # Send the message
                        bytes_sent = self.serial.write(msg.encode('utf-8'))
                        self.serial.flush()
                        self.stats['bytes_sent'] += bytes_sent
                        self.stats['packets_sent'] += 1
                        logger.debug(f"Sent: {msg.strip()} ({bytes_sent} bytes)")
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    logger.error(f"Input error: {e}")
        except Exception as e:
            logger.error(f"Echo test error: {e}")
            
        return True
    
    def _echo_reader(self):
        """Thread that reads from serial port in echo mode"""
        while self.running:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.5)
                    continue
                
                # Check if there's data to read
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    if data:
                        # Update stats
                        self.stats['bytes_received'] += len(data)
                        self.stats['packets_received'] += 1
                        
                        # Try to decode as UTF-8
                        try:
                            message = data.decode('utf-8').strip()
                            print(f"\nReceived: {message}")
                            print("> ", end='', flush=True)
                        except UnicodeDecodeError:
                            # Print as hex if not valid UTF-8
                            hex_data = binascii.hexlify(data).decode('ascii')
                            print(f"\nReceived (hex): {hex_data}")
                            print("> ", end='', flush=True)
                
                time.sleep(0.1)  # Small delay to prevent CPU usage
            
            except Exception as e:
                logger.error(f"Echo reader error: {e}")
                time.sleep(0.5)
    
    def stop(self):
        """Stop the SSH bridge"""
        self.running = False
        
        # Close all client connections
        for client in self.client_connections:
            try:
                client.close()
            except:
                pass
        self.client_connections = []
        
        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
            
        # Close serial connection
        if self.serial and self.serial.is_open:
            try:
                self.serial.close()
            except:
                pass
                
        logger.info("LoRa SSH bridge stopped")
        self._print_stats()
    
    def _accept_connections(self):
        """Thread that accepts new TCP connections"""
        while self.running:
            try:
                client_sock, addr = self.server_socket.accept()
                logger.info(f"New connection from {addr[0]}:{addr[1]}")
                
                self.client_connections.append(client_sock)
                
                # Start client handler thread
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_sock, addr),
                    name=f"Client-{addr[0]}:{addr[1]}"
                )
                client_thread.daemon = True
                client_thread.start()
                
            except socket.timeout:
                # This is expected due to the socket timeoutpy
                continue
            except Exception as e:
                if self.running:  # Only log if we're still supposed to be running
                    logger.error(f"Error accepting connection: {e}")
                break
    
    def _handle_client(self, client_sock, addr):
        """Handle data from a TCP client"""
        try:
            client_sock.settimeout(0.5)
            
            while self.running:
                try:
                    # Check if socket is still valid
                    if client_sock.fileno() < 0:
                        logger.debug(f"Socket {addr} no longer valid, closing connection")
                        break
                        
                    # Check if there's data to read - use try/except for select
                    try:
                        readable, _, _ = select.select([client_sock], [], [], 0.5)
                        
                        if client_sock in readable:
                            data = client_sock.recv(1024)
                            
                            if not data:  # Connection closed
                                logger.debug(f"No data received from {addr}, closing connection")
                                break
                                
                            # Forward data to serial port
                            if self.serial and self.serial.is_open:
                                bytes_written = self.serial.write(data)
                                self.serial.flush()
                                
                                # Update stats
                                self.stats['bytes_sent'] += bytes_written
                                self.stats['packets_sent'] += 1
                                
                                if self.debug:
                                    logger.debug(f"Forwarded {bytes_written} bytes to serial: " +
                                              f"{binascii.hexlify(data[:20])}" + 
                                              ("..." if len(data) > 20 else ""))
                    except (select.error, ValueError) as se:
                        logger.debug(f"Select error for {addr}: {se}")
                        # Socket likely invalid, break out of the loop
                        break
                                
                except socket.timeout:
                    continue
                except socket.error as e:
                    logger.debug(f"Socket error for {addr}: {e}")
                    break
                    
        except Exception as e:
            logger.error(f"Error handling client {addr}: {e}")
        finally:
            logger.info(f"Connection closed from {addr[0]}:{addr[1]}")
            try:
                client_sock.close()
                if client_sock in self.client_connections:
                    self.client_connections.remove(client_sock)
            except:
                pass
    
    def _serial_reader(self):
        """Thread that reads from serial and forwards to all clients"""
        buffer = bytearray()
        
        while self.running:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.5)
                    continue
                
                # Check if there's data to read
                if self.serial.in_waiting > 0:
                    # Read available data
                    data = self.serial.read(self.serial.in_waiting)
                    
                    if data:
                        # Update stats
                        self.stats['bytes_received'] += len(data)
                        self.stats['packets_received'] += 1
                        
                        if self.debug:
                            logger.debug(f"Received {len(data)} bytes from serial: " +
                                      f"{binascii.hexlify(data[:20])}" + 
                                      ("..." if len(data) > 20 else ""))
                        
                        # Forward to all connected clients
                        for client in list(self.client_connections):
                            try:
                                client.sendall(data)
                            except:
                                # Client likely disconnected
                                try:
                                    client.close()
                                    self.client_connections.remove(client)
                                except:
                                    pass
                
                # Short sleep to prevent CPU hogging
                time.sleep(0.01)
                
            except Exception as e:
                logger.error(f"Serial reader error: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _print_stats(self):
        """Print connection statistics"""
        duration = time.time() - self.stats['start_time']
        
        logger.info("=== Session Statistics ===")
        logger.info(f"Duration: {duration:.1f} seconds")
        logger.info(f"Bytes sent: {self.stats['bytes_sent']} ({self.stats['bytes_sent']/1024:.1f} KB)")
        logger.info(f"Bytes received: {self.stats['bytes_received']} ({self.stats['bytes_received']/1024:.1f} KB)")
        logger.info(f"Packets sent: {self.stats['packets_sent']}")
        logger.info(f"Packets received: {self.stats['packets_received']}")
        
        if duration > 0:
            tx_rate = (self.stats['bytes_sent'] * 8) / duration
            rx_rate = (self.stats['bytes_received'] * 8) / duration
            logger.info(f"TX rate: {tx_rate:.1f} bits/sec")
            logger.info(f"RX rate: {rx_rate:.1f} bits/sec")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    logger.info("Shutting down...")
    if 'bridge' in globals():
        bridge.stop()
    sys.exit(0)

def check_prereqs():
    """Check prerequisites for the script"""
    try:
        import serial
    except ImportError:
        logger.error("pyserial module not found. Install with: pip install pyserial")
        return False
    
    return True

def check_port_in_use(port):
    """Check if a port is already in use"""
    try:
        # Try to create a socket on the port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('0.0.0.0', port))
        s.close()
        return False  # Port is free
    except socket.error:
        return True  # Port is in use

def list_serial_ports():
    """List available serial ports"""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        if not ports:
            logger.info("No serial ports found")
            return
        
        logger.info("Available serial ports:")
        for port in ports:
            logger.info(f"  {port.device}: {port.description} [{port.hwid}]")
    except Exception as e:
        logger.error(f"Failed to list serial ports: {e}")

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='SSH over LoRa Bridge')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0',
                       help='Serial port for LoRa module (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=57600,
                       help='Baud rate for serial connection (default: 57600)')
    parser.add_argument('--ssh-port', type=int, default=8022,
                       help='Local port for SSH bridge (default: 8022)')
    parser.add_argument('--mode', type=str, choices=['bridge', 'ppp'], default='bridge',
                       help='Connection mode: bridge or ppp (default: bridge)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    parser.add_argument('--list-ports', action='store_true',
                       help='List available serial ports')
    parser.add_argument('--echo-test', action='store_true',
                       help='Run in echo test mode to verify LoRa communication')
    return parser.parse_args()

if __name__ == "__main__":
    # Check prerequisites
    if not check_prereqs():
        sys.exit(1)
        
    # Parse arguments
    args = parse_args()
    
    # Enable debug logging if requested
    if args.debug:
        logger.setLevel(logging.DEBUG)
    
    # List serial ports if requested
    if args.list_ports:
        list_serial_ports()
        sys.exit(0)
    
    # Check if the port is already in use
    if check_port_in_use(args.ssh_port):
        logger.error(f"Port {args.ssh_port} is already in use")
        logger.error("Try a different port with --ssh-port")
        sys.exit(1)
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    if args.mode == 'bridge':
        # Create and start the bridge
        bridge = LoRaSSHBridge(
            serial_port=args.port,
            baud_rate=args.baud,
            ssh_port=args.ssh_port,
            debug=args.debug,
            echo_test=args.echo_test
        )
        
        if bridge.start_server():
            if args.echo_test:
                # In echo test mode, the main thread is used for input
                try:
                    while True:
                        time.sleep(0.1)
                except KeyboardInterrupt:
                    signal_handler(None, None)
            else:
                logger.info("LoRa SSH bridge running. Press Ctrl+C to exit.")
                
                try:
                    # Keep the main thread alive
                    while True:
                        time.sleep(1)
                except KeyboardInterrupt:
                    signal_handler(None, None)
        else:
            logger.error("Failed to start LoRa SSH bridge")
            sys.exit(1)
            
    elif args.mode == 'ppp':
        # Establish PPP connection
        if not establish_direct_ppp():
            sys.exit(1)