import unittest
from src.hardware.gps_monitor import GPSMonitor

class TestGPSMonitor(unittest.TestCase):
    def setUp(self):
        self.gps_monitor = GPSMonitor()

    def test_get_gps_data(self):
        # Simulate getting GPS data
        gps_data = self.gps_monitor.get_gps_data()
        self.assertIsNotNone(gps_data)
        self.assertIn('lat', gps_data)
        self.assertIn('lon', gps_data)

    def test_calculate_heading(self):
        # Simulate two GPS positions
        start_position = (34.0522, -118.2437)  # Los Angeles
        end_position = (34.0523, -118.2436)    # Slightly north-east
        heading = self.gps_monitor.calculate_heading(start_position, end_position)
        self.assertIsInstance(heading, float)

    def test_invalid_gps_data(self):
        # Simulate invalid GPS data
        self.gps_monitor.gps_data = {'lat': None, 'lon': None}
        with self.assertRaises(ValueError):
            self.gps_monitor.calculate_heading((None, None), (None, None))

if __name__ == '__main__':
    unittest.main()