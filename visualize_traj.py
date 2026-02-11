import matplotlib.pyplot as plt

tx_coords = []
ty_coords = []
tz_coords = []
tx_estimated_coords = []
ty_estimated_coords = []
tz_estimated_coords = []

with open("traj.dat", "r") as file:
    lines = file.readlines()


for line in lines[2:]:
    parts = line.split()
    if len(parts) >= 10:  # Ensure there are enough parts
        tx_coords.append(float(parts[3]))
        ty_coords.append(float(parts[4]))
        tz_coords.append(float(parts[5]))



#show my poses 
with open("poses.dat", "r") as file:
    lines = file.readlines()


for line in lines:
    parts = line.split()
    if len(parts) >= 10:  
        tx_estimated_coords.append(float(parts[3]))
        ty_estimated_coords.append(float(parts[4]))
        tz_estimated_coords.append(float(parts[5]))

#------------

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(tx_coords, ty_coords, tz_coords, label="GT Trajectory", color = "blue")
ax.plot(tx_estimated_coords, ty_estimated_coords, tz_estimated_coords, label="Estimated Trajectory", color = "red",linestyle = "dashed")

ax.set_xlabel("X (tx)")
ax.set_ylabel("Y (ty)")
ax.set_zlabel("Z (tz)")

ax.legend()

plt.show()
