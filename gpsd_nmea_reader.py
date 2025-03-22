#!/usr/bin/env python3
# filepath: /home/jay/projects/grassSmoking/gpsd_nmea_reader.py

import gps
import signal
import sys
import time

# Flag to control the main loop
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    global running
    print("\nStopping GPSD NMEA reader...")
    running = False

def main():
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("Starting GPSD NMEA sentence capture...")
    print("Press Ctrl+C to exit")
    print("-" * 60)
    
    try:
        # Connect to gpsd
        gpsd = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NMEA | gps.WATCH_NEWSTYLE)
        
        # Main loop
        while running:
            # Get next report (blocks until data is available)
            report = gpsd.next()
            
            # Check for NMEA sentences
            if report['class'] == 'NMEA':
                print(f"{report.get('device', 'unknown')}: {report.get('nmea', '')}")
            
    except KeyboardInterrupt:
        # This should be caught by the signal handler
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'gpsd' in locals():
            gpsd.close()

if __name__ == "__main__":
    main()