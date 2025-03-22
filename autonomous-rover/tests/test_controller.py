import unittest
import numpy as np
from src.controllers.stanley_controller import add_extension_waypoint, haversine_distance

class TestStanleyController(unittest.TestCase):

    def setUp(self):
        self.waypoints = np.array([
            [37.7749, -122.4194],  # San Francisco
            [37.7750, -122.4184],  # Slightly east
            [37.7751, -122.4174]   # Slightly further east
        ])

    def test_add_extension_waypoint(self):
        extended_waypoints = add_extension_waypoint(self.waypoints, extension_distance=1.0)
        self.assertEqual(extended_waypoints.shape[0], self.waypoints.shape[0] + 1)
        
        # Check the last waypoint is the extension
        last_waypoint = extended_waypoints[-1]
        self.assertAlmostEqual(haversine_distance(last_waypoint, self.waypoints[-1]), 1.0, delta=0.1)

    def test_haversine_distance(self):
        coord1 = (37.7749, -122.4194)  # San Francisco
        coord2 = (37.7750, -122.4184)  # Slightly east
        distance = haversine_distance(coord1, coord2)
        self.assertGreater(distance, 0)

if __name__ == '__main__':
    unittest.main()