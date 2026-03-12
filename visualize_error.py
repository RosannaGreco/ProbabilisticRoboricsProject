import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import re

x_err = []
y_err = []
z_err = []
roll = []
pitch = []
yaw = []


with open("output/error.dat", "r") as file:
    lines = file.readlines()

def get_roll_pitch_yaw(qx,qy,qz,qw):
    r = np.arctan2(2*(qw*qx + qy*qz), 1- 2*(qx*qx + qy*qy))
    p = np.arcsin(2*(qw*qy - qz*qx))
    y = np.arctan2(2*(qw*qz + qx*qy), 1- 2*(qy*qy + qz*qz))
    return r, p, y

for line in lines:
    parts = line.split()
    if len(parts) >= 10:  
        x_err.append(float(parts[3]))
        y_err.append(float(parts[4]))
        z_err.append(float(parts[5]))

        qx = float(parts[6])
        qy = float(parts[7])
        qz = float(parts[8])
        qw = float(parts[9])
        
        r,p,y = get_roll_pitch_yaw(qx,qy,qz,qw)
        roll.append(r)
        pitch.append(p)
        yaw.append(y)

fig = plt.figure()
ax = fig.add_subplot(111)



#ax.plot(x_err, label="x error", color="blue")
#ax.plot(y_err, label="y error", color="green")
#ax.plot(z_err, label="z error", color="red")
#ax.plot(roll, label="roll", color="orange")
#ax.plot(pitch, label="pitch", color="blueviolet")
ax.plot(yaw, label="yaw", color="magenta")

ax.set_xlabel("epoch")
ax.set_ylabel("error")
ax.ticklabel_format(style='plain', axis='y')
ax.set_ylim(-0.001, 0.001)
ax.legend()

plt.show()