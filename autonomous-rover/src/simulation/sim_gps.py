import numpy as np
import time

class SimulatedGPS:
    def __init__(self, initial_latitude=0.0, initial_longitude=0.0, noise_level=0.0001):
        self.latitude = initial_latitude
        self.longitude = initial_longitude
        self.noise_level = noise_level

    def get_gps_data(self):
        # Simulate GPS data with some noise
        lat_noise = np.random.normal(0, self.noise_level)
        lon_noise = np.random.normal(0, self.noise_level)
        
        self.latitude += lat_noise
        self.longitude += lon_noise
        
        return self.latitude, self.longitude

def main():
    gps_simulator = SimulatedGPS(initial_latitude=37.7749, initial_longitude=-122.4194)
    
    try:
        while True:
            latitude, longitude = gps_simulator.get_gps_data()
            print(f"Simulated GPS Data: Latitude: {latitude}, Longitude: {longitude}")
            time.sleep(1)  # Simulate a delay between GPS readings
    except KeyboardInterrupt:
        print("GPS simulation stopped.")

if __name__ == "__main__":
    main()