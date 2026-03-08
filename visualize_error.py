import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import re

x_err = []
y_err = []
z_err = []
qx_err = []
qy_err = []
qz_err = []
qw_err = []

with open("output/error.dat", "r") as file:
    lines = file.readlines()

for line in lines:
    parts = line.split()
    if len(parts) >= 10:  
        x_err.append(float(parts[3]))
        y_err.append(float(parts[4]))
        z_err.append(float(parts[5]))
        qx_err.append(float(parts[6]))
        qy_err.append(float(parts[7]))
        qz_err.append(float(parts[8]))
        qw_err.append(float(parts[9]))

fig = plt.figure()
ax = fig.add_subplot(111)

#fig, ax = plt.subplots()

ax.plot(qz_err, label="qz error", color="magenta")

ax.set_xlabel("epoch")
ax.set_ylabel("error")
ax.ticklabel_format(style='plain', axis='y')
ax.set_ylim(-0.001, 0.001)
ax.legend()

plt.show()