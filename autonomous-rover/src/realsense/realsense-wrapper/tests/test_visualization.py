import unittest
from src.visualization.text_renderer import TextRenderer
from src.visualization.visual_renderer import VisualRenderer

class TestTextRenderer(unittest.TestCase):
    def setUp(self):
        self.renderer = TextRenderer()

    def test_render_empty_coverage(self):
        coverage = [0] * 64
        output = self.renderer.render(coverage)
        expected_output = " " * 64  # Assuming empty coverage renders as spaces
        self.assertEqual(output, expected_output)

    def test_render_full_coverage(self):
        coverage = [25] * 64  # Simulating full coverage
        output = self.renderer.render(coverage)
        expected_output = "W" * 64  # Assuming full coverage renders as 'W'
        self.assertEqual(output, expected_output)

class TestVisualRenderer(unittest.TestCase):
    def setUp(self):
        self.renderer = VisualRenderer()

    def test_visualize_depth_data(self):
        depth_data = [[0.5] * 640 for _ in range(480)]  # Simulated depth data
        result = self.renderer.visualize(depth_data)
        self.assertIsNotNone(result)  # Ensure that the visualization result is not None

if __name__ == '__main__':
    unittest.main()