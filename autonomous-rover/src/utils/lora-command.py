#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/autonomous-rover/src/utils/lora_command.py

import serial
import time
import sys
import threading
import argparse
import subprocess
import logging
import os
import json
import base64

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('LoRa-Command')

class LoRaCommandShell:
    def __init__(self, serial_port, baud_rate=57600, debug=False, server_mode=False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.debug = debug
        self.server_mode = server_mode
        self.serial = None
        self.running = False
        self.command_id = 0
        self.pending_commands = {}
        
        if self.debug:
            logger.setLevel(logging.DEBUG)
    
    def start(self):
        """Start the command shell"""
        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                bytesize=8,
                parity='N',
                stopbits=1
            )
            logger.info(f"Connected to LoRa on {self.serial_port} at {self.baud_rate} baud")
            
            # Start reader thread
            self.running = True
            reader_thread = threading.Thread(target=self._serial_reader)
            reader_thread.daemon = True
            reader_thread.start()
            
            if self.server_mode:
                # Server mode - wait for commands and execute them
                logger.info("Running in server mode - waiting for commands")
                try:
                    while self.running:
                        time.sleep(1)
                except KeyboardInterrupt:
                    self.running = False
            else:
                # Client mode - provide interactive shell
                self._send_command("ping", {})
                logger.info("Remote command shell ready. Type commands or 'help' for assistance.")
                try:
                    self._command_loop()
                except KeyboardInterrupt:
                    self.running = False
                    
            return True
        except Exception as e:
            logger.error(f"Failed to start: {e}")
            return False
    
    def stop(self):
        """Stop the command shell"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        logger.info("LoRa command shell stopped")
    
    def _command_loop(self):
        """Interactive command loop for client mode"""
        while self.running:
            try:
                cmd_str = input("> ")
                if not cmd_str:
                    continue
                
                cmd_parts = cmd_str.strip().split()
                cmd_name = cmd_parts[0].lower()
                
                if cmd_name == 'exit' or cmd_name == 'quit':
                    logger.info("Exiting...")
                    self.running = False
                    break
                
                elif cmd_name == 'help':
                    print("Available commands:")
                    print("  exec <command> - Execute a shell command")
                    print("  ls [path] - List directory contents")
                    print("  cat <file> - Display file contents")
                    print("  status - Get system status")
                    print("  ping - Test connection")
                    print("  exit/quit - Exit the shell")
                
                elif cmd_name == 'ping':
                    self._send_command("ping", {})
                    
                elif cmd_name == 'status':
                    self._send_command("status", {})
                    
                elif cmd_name == 'ls':
                    path = cmd_parts[1] if len(cmd_parts) > 1 else "."
                    self._send_command("ls", {"path": path})
                    
                elif cmd_name == 'cat':
                    if len(cmd_parts) < 2:
                        print("Error: Missing file path")
                        continue
                    self._send_command("cat", {"path": cmd_parts[1]})
                    
                elif cmd_name == 'exec':
                    if len(cmd_parts) < 2:
                        print("Error: Missing command")
                        continue
                    self._send_command("exec", {"cmd": " ".join(cmd_parts[1:])})
                    
                else:
                    print(f"Unknown command: {cmd_name}")
                    print("Type 'help' for available commands")
                
            except KeyboardInterrupt:
                self.running = False
                break
            except Exception as e:
                logger.error(f"Command error: {e}")
    
    def _send_command(self, cmd_type, data):
        """Send a command to the remote system"""
        if not self.serial or not self.serial.is_open:
            logger.error("Serial port not open")
            return
            
        self.command_id += 1
        cmd_obj = {
            "id": self.command_id,
            "type": cmd_type,
            "data": data,
            "timestamp": time.time()
        }
        
        try:
            # Store in pending commands
            self.pending_commands[self.command_id] = cmd_obj
            
            # Convert to JSON and send
            json_data = json.dumps(cmd_obj) + "\n"
            bytes_sent = self.serial.write(json_data.encode('utf-8'))
            self.serial.flush()
            
            if self.debug:
                logger.debug(f"Sent: {json_data.strip()}")
            
            return self.command_id
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            return None
    
    def _serial_reader(self):
        """Read and process data from serial port"""
        buffer = ""
        
        while self.running:
            try:
                if not self.serial or not self.serial.is_open:
                    time.sleep(0.5)
                    continue
                    
                # Check if there's data to read
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting).decode('utf-8', errors='replace')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        self._process_message(line)
                        
                # Short sleep to prevent CPU hogging
                time.sleep(0.05)
                
            except Exception as e:
                logger.error(f"Serial reader error: {e}")
                time.sleep(1)  # Wait before retrying
    
    def _process_message(self, message):
        """Process a received message"""
        if not message.strip():
            return
            
        try:
            # Try to parse as JSON
            data = json.loads(message)
            
            if 'id' not in data:
                logger.warning(f"Received message without ID: {message}")
                return
                
            if self.server_mode:
                # Server mode - execute the command
                self._execute_command(data)
            else:
                # Client mode - display the response
                self._display_response(data)
                
        except json.JSONDecodeError:
            logger.warning(f"Received non-JSON data: {message}")
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    def _execute_command(self, cmd_data):
        """Execute a command on the server"""
        try:
            cmd_id = cmd_data.get('id')
            cmd_type = cmd_data.get('type')
            data = cmd_data.get('data', {})
            
            response = {
                "id": cmd_id,
                "type": "response",
                "success": False,
                "data": {}
            }
            
            if cmd_type == "ping":
                response["success"] = True
                response["data"] = {"status": "ok", "timestamp": time.time()}
                
            elif cmd_type == "status":
                # Get system status
                try:
                    uptime = subprocess.check_output("uptime", shell=True).decode().strip()
                    memory = subprocess.check_output("free -h", shell=True).decode().strip()
                    disk = subprocess.check_output("df -h /", shell=True).decode().strip()
                    response["success"] = True
                    response["data"] = {
                        "uptime": uptime, 
                        "memory": memory, 
                        "disk": disk
                    }
                except Exception as e:
                    response["data"] = {"error": str(e)}
                    
            elif cmd_type == "ls":
                # List directory
                path = data.get('path', '.')
                try:
                    # Safety check
                    if '..' in path or path.startswith('/'):
                        response["data"] = {"error": "Path not allowed"}
                    else:
                        files = os.listdir(path)
                        file_details = []
                        for f in files:
                            try:
                                full_path = os.path.join(path, f)
                                stat = os.stat(full_path)
                                is_dir = os.path.isdir(full_path)
                                size = stat.st_size
                                file_details.append({
                                    "name": f,
                                    "is_dir": is_dir,
                                    "size": size,
                                    "mtime": stat.st_mtime
                                })
                            except:
                                file_details.append({"name": f, "error": "stat failed"})
                        response["success"] = True
                        response["data"] = {"files": file_details}
                except Exception as e:
                    response["data"] = {"error": str(e)}
                    
            elif cmd_type == "cat":
                # Show file contents
                path = data.get('path')
                try:
                    # Safety check
                    if '..' in path or path.startswith('/'):
                        response["data"] = {"error": "Path not allowed"}
                    else:
                        with open(path, 'r') as f:
                            content = f.read(8192)  # Limit to 8KB
                        response["success"] = True
                        response["data"] = {"content": content}
                except Exception as e:
                    response["data"] = {"error": str(e)}
                    
            elif cmd_type == "exec":
                # Execute a command
                cmd = data.get('cmd')
                try:
                    # Very basic security - don't allow certain commands
                    if any(x in cmd for x in [';', '&&', '||', '`']):
                        response["data"] = {"error": "Command contains disallowed characters"}
                    else:
                        output = subprocess.check_output(cmd, shell=True, timeout=5).decode()
                        response["success"] = True
                        response["data"] = {"output": output}
                except subprocess.CalledProcessError as e:
                    response["data"] = {"error": f"Exit code {e.returncode}: {e.output.decode()}"}
                except Exception as e:
                    response["data"] = {"error": str(e)}
            
            else:
                response["data"] = {"error": f"Unknown command: {cmd_type}"}
                
            # Send the response
            response_json = json.dumps(response) + "\n"
            self.serial.write(response_json.encode('utf-8'))
            self.serial.flush()
            
        except Exception as e:
            logger.error(f"Error executing command: {e}")
            # Try to send error response
            try:
                error_resp = {
                    "id": cmd_data.get('id', 0),
                    "type": "response",
                    "success": False,
                    "data": {"error": str(e)}
                }
                self.serial.write((json.dumps(error_resp) + "\n").encode('utf-8'))
                self.serial.flush()
            except:
                pass
    
    def _display_response(self, response):
        """Display a response from the server"""
        resp_id = response.get('id')
        resp_type = response.get('type')
        success = response.get('success', False)
        data = response.get('data', {})
        
        # Remove from pending commands
        if resp_id in self.pending_commands:
            cmd = self.pending_commands.pop(resp_id)
            cmd_type = cmd.get('type')
            
            if resp_type == "response":
                if cmd_type == "ping":
                    if success:
                        print("PING successful - connection working")
                    else:
                        print(f"PING failed: {data.get('error', 'Unknown error')}")
                        
                elif cmd_type == "status":
                    if success:
                        print("\n=== System Status ===")
                        print(data.get('uptime', 'No uptime data'))
                        print("\n--- Memory ---")
                        print(data.get('memory', 'No memory data'))
                        print("\n--- Disk ---")
                        print(data.get('disk', 'No disk data'))
                    else:
                        print(f"Status request failed: {data.get('error', 'Unknown error')}")
                        
                elif cmd_type == "ls":
                    if success:
                        files = data.get('files', [])
                        print(f"\nDirectory listing ({len(files)} items):")
                        for f in files:
                            if f.get('is_dir'):
                                print(f"[DIR]  {f['name']}/")
                            else:
                                size = f.get('size', 0)
                                if size < 1024:
                                    size_str = f"{size}B"
                                elif size < 1024 * 1024:
                                    size_str = f"{size/1024:.1f}K"
                                else:
                                    size_str = f"{size/(1024*1024):.1f}M"
                                print(f"[FILE] {f['name']} ({size_str})")
                    else:
                        print(f"LS failed: {data.get('error', 'Unknown error')}")
                        
                elif cmd_type == "cat":
                    if success:
                        print("\n=== File Content ===")
                        print(data.get('content', ''))
                        print("=====================")
                    else:
                        print(f"CAT failed: {data.get('error', 'Unknown error')}")
                        
                elif cmd_type == "exec":
                    if success:
                        print("\n=== Command Output ===")
                        print(data.get('output', ''))
                        print("=====================")
                    else:
                        print(f"Command execution failed: {data.get('error', 'Unknown error')}")
                        
            else:
                print(f"Unknown response type: {resp_type}")
        else:
            print(f"Received response for unknown command ID {resp_id}")

def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='LoRa Remote Command Shell')
    parser.add_argument('--port', type=str, required=True,
                       help='Serial port (e.g., /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=57600,
                       help='Baud rate (default: 57600)')
    parser.add_argument('--server', action='store_true',
                       help='Run in server mode (on the rover)')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug logging')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    
    shell = LoRaCommandShell(
        serial_port=args.port,
        baud_rate=args.baud,
        debug=args.debug,
        server_mode=args.server
    )
    
    try:
        if shell.start():
            pass  # The start method handles the main loop
        else:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        shell.stop()