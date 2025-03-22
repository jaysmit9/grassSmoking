import math
from shapely.geometry import Point, Polygon, LineString
import matplotlib.pyplot as plt

def create_parallel_lines(polygon, start_point, angle_point, line_spacing):
    # Calculate the angle between the start_point and angle_point
    angle = math.atan2(angle_point[1] - start_point[1], angle_point[0] - start_point[0])
    print(f"Calculated angle: {math.degrees(angle)} degrees")

    # Create empty lists to store the lines
    lines_inside = []
    lines_outside = []

    # Function to generate a line segment
    def generate_line(start, angle, length):
        return [
            (start[0], start[1]),
            (start[0] + length * math.cos(angle), start[1] + length * math.sin(angle))
        ]

    # Function to offset a point
    def offset_point(point, angle, distance):
        return (
            point[0] + distance * math.cos(angle),
            point[1] + distance * math.sin(angle)
        )

    # Generate the initial line
    current_point = start_point
    while polygon.contains(Point(current_point)):
        line_start = current_point
        line_end = generate_line(current_point, angle, 0.01)  # Increase the length to make lines more visible
        print(f"Checking line from {line_start} to {line_end[1]}")  # Debugging statement
        if not polygon.contains(Point(line_end[1])):
            print(f"End point {line_end[1]} not inside polygon")  # Debugging statement
            break

        line = LineString([line_start, line_end[1]])
        if polygon.contains(line):
            lines_inside.append(line)
            print(f"Generated line: {line_start} to {line_end[1]}")  # Debugging statement
        else:
            lines_outside.append(line)
            print(f"Line not inside polygon: {line_start} to {line_end[1]}")  # Debugging statement

        current_point = line_end[1]

    # Generate additional parallel lines by offsetting the initial line
    for i in range(1, 100):  # Adjust the range as needed to generate more lines
        offset = i * line_spacing
        offset_line_start = offset_point(start_point, angle + math.pi / 2, offset)
        offset_line_end = generate_line(offset_line_start, angle, 0.01)[1]  # Generate a line segment from the offset point
        print(f"Checking offset line from {offset_line_start} to {offset_line_end}")  # Debugging statement
        if polygon.contains(Point(offset_line_start)) and polygon.contains(Point(offset_line_end)):
            line = LineString([offset_line_start, offset_line_end])
            if polygon.contains(line):
                lines_inside.append(line)
                print(f"Generated offset line: {offset_line_start} to {offset_line_end}")  # Debugging statement
            else:
                lines_outside.append(line)
                print(f"Offset line not inside polygon: {offset_line_start} to {offset_line_end}")  # Debugging statement
        else:
            print(f"Offset points not inside polygon: {offset_line_start}, {offset_line_end}")  # Debugging statement

    return lines_inside, lines_outside

# Define the polygon and points
polygon_coords = [(34.151997, -77.8668216), (34.152097, -77.8667216), (34.152197, -77.8668416), (34.152097, -77.8669416)]
polygon = Polygon(polygon_coords)
start_point = (34.151997, -77.8668216)
angle_point = (34.152097, -77.8667216)
line_spacing = 3 / 364000  # Line spacing in degrees for approximately 3 feet apart

# Create parallel lines
lines_inside, lines_outside = create_parallel_lines(polygon, start_point, angle_point, line_spacing)

# Plot the polygon and lines using Matplotlib
fig, ax = plt.subplots()
polygon_coords.append(polygon_coords[0])  # Close the polygon
polygon_x, polygon_y = zip(*polygon_coords)
ax.plot(polygon_x, polygon_y, 'b-', label='Polygon')

# Plot lines inside the polygon
if lines_inside:
    line_x, line_y = lines_inside[0].xy
    ax.plot(line_x, line_y, 'r-', label='Line Inside')
    for line in lines_inside[1:]:
        line_x, line_y = line.xy
        ax.plot(line_x, line_y, 'r-')

# Plot lines outside the polygon
if lines_outside:
    line_x, line_y = lines_outside[0].xy
    ax.plot(line_x, line_y, 'g--', label='Line Outside')
    for line in lines_outside[1:]:
        line_x, line_y = line.xy
        ax.plot(line_x, line_y, 'g--')
        print("Lines outside the polygon")

ax.set_aspect('equal')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.title('Parallel Lines Inside and Outside Polygon')
plt.legend()
plt.savefig('parallel_lines.png')
plt.show()