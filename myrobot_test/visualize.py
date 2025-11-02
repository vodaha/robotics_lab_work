# Must-have libraries
import matplotlib.pyplot as plot
import csv
import os

# Declare x and y axis as an array
x_axis = []
y_axis = []

# Access csv file
with open('robot_path.csv', 'r') as file:
    csv_reader = csv.reader(file)
    headers = next(csv_reader)  # the first row represents all the headers

    # Find headers corresponding to x and y coordinates
    x = headers.index('x') if 'x' in headers else 1  # if a header x was not found pick second column as x coordinates
    y = headers.index('y') if 'y' in headers else 2  # same for a header y but pick 3rd column

    # Store each row in the corresponding array
    for row in csv_reader:
        x_axis.append(float(row[x]))
        y_axis.append(float(row[y]))

# Plot the coordinates
plot.figure(figsize=(10, 10))
plot.plot(x_axis, y_axis, 'b-', linewidth=2, label="Robot's path")
plot.plot(x_axis[0], y_axis[0], 'ro', markersize=8, label='Start')
plot.plot(x_axis[-1], y_axis[-1], 'go', markersize=8, label='End')

# Plot the grid
plot.xlabel('X-Axis')
plot.ylabel('Y-Axis')
plot.title("Robot's path")
plot.grid(True, alpha=0.3)
plot.axis('equal')
plot.legend()
plot.show()