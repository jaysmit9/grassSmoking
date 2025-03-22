import unittest
from src.camera import RealSenseCamera

class TestRealSenseCamera(unittest.TestCase):

    def setUp(self):
        self.camera = RealSenseCamera()

    def test_initialization(self):
        self.assertIsNotNone(self.camera.pipeline)
        self.assertIsNotNone(self.camera.config)

    def test_get_depth_data(self):
        depth_data = self.camera.get_depth_data()
        self.assertIsNotNone(depth_data)
        self.assertEqual(depth_data.shape[0], 480)  # Assuming 480p resolution
        self.assertEqual(depth_data.shape[1], 640)  # Assuming 640p resolution

    def tearDown(self):
        self.camera.stop()

if __name__ == '__main__':
    unittest.main()