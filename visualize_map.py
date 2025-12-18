import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

x_coords = []
y_coords = []
z_coords = []

with open("map.dat", "r") as file:
    lines = file.readlines()

pattern = re.compile(
    r"id:\s*(\d+)\s*point:\s*([-+]?\d*\.\d+|\d+)\s*([-+]?\d*\.\d+|\d+)\s*([-+]?\d*\.\d+|\d+)"
)

for line in lines[1:]:
    match = pattern.match(line)
    if match:
        x_coords.append(float(match.group(2)))
        y_coords.append(float(match.group(3)))
        z_coords.append(float(match.group(4)))

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(x_coords, y_coords, z_coords, c="r", marker="o", label="Landmark Position")

ax.legend()
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

plt.show()
