import math
import numpy as np
from shapely.geometry import Point, Polygon, LineString

class BasicPathPlanner:
    def __init__(self, polygon):
        self.polygon = Polygon(polygon)
        self.grid_size = 3 / 364000  # Grid size in degrees for approximately 3 feet apart
        self.path = []

    def is_inside_polygon(self, x, y):
        point = Point(x, y)
        # Use a larger buffer to handle precision issues
        inside = self.polygon.buffer(1e-6).contains(point)
        print(f"Point ({x}, {y}) inside polygon: {inside}")  # Debugging statement
        return inside

    def plan_basic_path(self, start_point, angle_point):
        if not start_point or not angle_point:
            print("Start point and angle point are required.")
            return []

        # Calculate the angle between the start_point and angle_point
        angle = math.atan2(angle_point[1] - start_point[1], angle_point[0] - start_point[0])
        print(f"Calculated angle: {math.degrees(angle)} degrees")  # Debugging statement

        # Generate parallel lines
        basic_path = []
        offset_distance = 3 / 364000  # Offset distance in degrees for approximately 3 feet apart

        def generate_line(start, angle, length):
            return (
                start[0] + length * math.cos(angle),
                start[1] + length * math.sin(angle)
            )

        def offset_point(point, angle, distance):
            return (
                point[0] + distance * math.cos(angle),
                point[1] + distance * math.sin(angle)
            )

        # Generate the initial line
        current_point = start_point
        while self.is_inside_polygon(current_point[0], current_point[1]):
            line_start = current_point
            line_end = generate_line(current_point, angle, self.grid_size)
            if not self.is_inside_polygon(line_end[0], line_end[1]):
                break

            line = LineString([line_start, line_end])
            if self.polygon.contains(line):
                basic_path.append((line_start[0], line_start[1]))
                basic_path.append((line_end[0], line_end[1]))

            current_point = line_end

        # Generate additional parallel lines by offsetting the initial line
        for i in range(1, 100):  # Adjust the range as needed to generate more lines
            offset = i * offset_distance
            offset_line_start = offset_point(start_point, angle + math.pi / 2, offset)
            offset_line_end = offset_point(angle_point, angle + math.pi / 2, offset)
            if self.is_inside_polygon(offset_line_start[0], offset_line_start[1]) and self.is_inside_polygon(offset_line_end[0], offset_line_end[1]):
                basic_path.append((offset_line_start[0], offset_line_start[1]))
                basic_path.append((offset_line_end[0], offset_line_end[1]))

        self.path = basic_path
        print(f"Planned basic path length: {len(self.path)}")  # Debugging statement

    def get_path(self):
        return self.path