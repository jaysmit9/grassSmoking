import numpy as np
from scipy.spatial import ConvexHull
import math

class GridBasedSweepCoverage:
    def __init__(self, polygon):
        self.polygon = polygon
        self.grid_size = 3 / 364000  # Grid size in degrees for approximately 3 feet apart
        self.path = []

    def generate_grid(self):
        min_x = min([p[0] for p in self.polygon])
        max_x = max([p[0] for p in self.polygon])
        min_y = min([p[1] for p in self.polygon])
        max_y = max([p[1] for p in self.polygon])

        x_range = np.arange(min_x, max_x, self.grid_size)
        y_range = np.arange(min_y, max_y, self.grid_size)

        grid_points = []
        for x in x_range:
            for y in y_range:
                if self.is_inside_polygon(x, y):
                    grid_points.append((x, y))

        return grid_points

    def is_inside_polygon(self, x, y):
        hull = ConvexHull(self.polygon)
        new_point = np.array([x, y])
        inside = all(np.dot(eq[:-1], new_point) + eq[-1] <= 0 for eq in hull.equations)
        return inside

    def plan_path(self, start_point=None, angle_point=None):
        grid_points = self.generate_grid()
        if not grid_points:
            print("No grid points generated.")
            return

        # Sort grid points by y, then by x
        grid_points.sort(key=lambda p: (p[1], p[0]))

        if start_point:
            # Find the closest grid point to the start_point
            start_point = min(grid_points, key=lambda p: (p[0] - start_point[0])**2 + (p[1] - start_point[1])**2)
        else:
            # Default to the bottom-left corner
            start_point = min(grid_points, key=lambda p: (p[0], p[1]))

        if angle_point:
            # Calculate the angle between the start_point and angle_point
            angle = math.atan2(angle_point[1] - start_point[1], angle_point[0] - start_point[0])
            print(f"Calculated angle: {math.degrees(angle)} degrees")  # Debugging statement
        else:
            # Default to horizontal lines
            angle = 0
            print(f"Default angle: {math.degrees(angle)} degrees")  # Debugging statement

        # Adjust the starting point to begin 3 feet in from the corner
        start_point = (
            start_point[0] + self.grid_size * math.cos(angle),
            start_point[1] + self.grid_size * math.sin(angle)
        )
        print(f"Adjusted start point: {start_point}")  # Debugging statement

        # Create zigzag path with parallel lines starting from the adjusted start_point
        path = [start_point]
        current_point = start_point
        direction = 1  # 1 for right, -1 for left

        while grid_points:
            # Find the next point in the current direction
            next_point = min(grid_points, key=lambda p: (p[1] - (current_point[1] + direction * self.grid_size * math.sin(angle)))**2 + (p[0] - (current_point[0] + direction * self.grid_size * math.cos(angle)))**2)
            path.append(next_point)
            grid_points.remove(next_point)
            previous_point = current_point
            current_point = next_point

            # Calculate the angle between the previous point and the current point
            segment_angle = math.atan2(current_point[1] - previous_point[1], current_point[0] - previous_point[0])
            print(f"Current point: {current_point}, Direction: {direction}, Segment angle: {math.degrees(segment_angle)} degrees")  # Debugging statement

            # Check if we need to change direction
            if not any(abs(p[1] - (current_point[1] + self.grid_size * math.sin(angle))) < self.grid_size / 2 for p in grid_points):
                direction *= -1
                # Change direction 90 degrees off the preferred angle
                angle += math.pi / 2
                next_row_points = [p for p in grid_points if abs(p[1] - (current_point[1] + self.grid_size * math.sin(angle))) < self.grid_size / 2]
                if next_row_points:
                    next_point = min(next_row_points, key=lambda p: (p[1] - (current_point[1] + direction * self.grid_size * math.sin(angle)))**2 + (p[0] - (current_point[0] + direction * self.grid_size * math.cos(angle)))**2)
                    path.append(next_point)
                    grid_points.remove(next_point)
                    previous_point = current_point
                    current_point = next_point
                    segment_angle = math.atan2(current_point[1] - previous_point[1], current_point[0] - previous_point[0])
                    print(f"Changed direction. Current point: {current_point}, Direction: {direction}, Segment angle: {math.degrees(segment_angle)} degrees")  # Debugging statement

        self.path = path
        # print(f"Planned path: {self.path}")  # Commented out the generated path debug statement
        print(f"Planned path length: {len(self.path)}")  # Debugging statement

    def plan_basic_path(self, start_point=None, angle_point=None):
        if not start_point or not angle_point:
            print("Start point and angle point are required.")
            return []

        # Calculate the angle between the start_point and angle_point
        angle = math.atan2(angle_point[1] - start_point[1], angle_point[0] - start_point[0])
        print(f"Calculated angle: {math.degrees(angle)} degrees")  # Debugging statement

        # Generate lines parallel to the angle segment
        path = []
        current_point = start_point
        while self.is_inside_polygon(current_point[0], current_point[1]):
            path.append(current_point)
            next_point = (
                current_point[0] + self.grid_size * math.cos(angle),
                current_point[1] + self.grid_size * math.sin(angle)
            )
            if not self.is_inside_polygon(next_point[0], next_point[1]):
                break
            current_point = next_point

        # Add lines three feet apart, ending three feet before the edge of the polygon
        offset_angle = angle + math.pi / 2
        basic_path = []
        for point in path:
            line_start = (
                point[0] + 3 * self.grid_size * math.cos(offset_angle),
                point[1] + 3 * self.grid_size * math.sin(offset_angle)
            )
            line_end = (
                point[0] - 3 * self.grid_size * math.cos(offset_angle),
                point[1] - 3 * self.grid_size * math.sin(offset_angle)
            )
            if self.is_inside_polygon(line_start[0], line_start[1]) and self.is_inside_polygon(line_end[0], line_end[1]):
                basic_path.append(line_start)
                basic_path.append(line_end)

        self.path = basic_path
        print(f"Planned basic path length: {len(self.path)}")  # Debugging statement

    def get_path(self):
        return self.path

def main():
    polygon = [(34.151997, -77.8668216), (34.152097, -77.8667216), (34.152197, -77.8668416), (34.152097, -77.8669416)]
    planner = GridBasedSweepCoverage(polygon)
    start_point = (34.151997, -77.8668216)  # User-defined starting point
    angle_point = (34.152097, -77.8667216)  # User-defined angle point
    planner.plan_basic_path(start_point=start_point, angle_point=angle_point)
    path = planner.get_path()

    import matplotlib.pyplot as plt
    plt.plot([p[0] for p in polygon], [p[1] for p in polygon], 'b-')
    plt.plot([p[0] for p in path], [p[1] for p in path], 'r-')
    plt.show()

if __name__ == '__main__':
    main()