import gpsd

try:
    gpsd.connect()
    packet = gpsd.get_current()
    if packet.mode >= 2:  # Check if a fix is available
        print(f"Latitude: {packet.lat:.6f}, Longitude: {packet.lon:.6f}")
    else:
        print("No fix available")
except ConnectionError:
    print("gpsd is not running or is not accessible")
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    gpsd.stop()
