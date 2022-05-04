import pickle
import numpy as np
from matplotlib import pyplot as plt


POINTCLOUD_IDX = 27
N_VECTORS = 5


with open('../logs/flight2.pickle', 'rb') as file:
    data = pickle.load(file)

vx = np.array(data['vx']).reshape(-1, 1)
vy = np.array(data['vy']).reshape(-1, 1)
vz = np.array(data['vz']).reshape(-1, 1)

v = np.append(vx, vy, axis=1)
v = np.append(v, vz, axis=1)

plt.plot(data['time'], np.linalg.norm(v, axis=1), label='v')
plt.plot(data['time'], data['x'], label='x')
plt.plot(data['time'], data['y'], label='y')
plt.plot(data['time'], data['z'], label='z')
plt.xlabel('Время с начала миссии, с')
plt.ylabel('Координата, м')
plt.legend()
plt.show()

fig = plt.figure(figsize=(10, 10))
ax = fig.gca(projection='3d')
ax.scatter(data['x'], data['y'], data['z'], color='green')

for i, trajectory in enumerate(data['local_trajectories']):
    if 27 <= i <= 27:
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
        ax.text(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2]+0.4, s=str(i))
        ax.scatter(data['coord_begin_execution'][i][0], data['coord_begin_execution'][i][1], data['coord_begin_execution'][i][2], color='black')
        # Plot velocity setpoint vectors
        step = len(data['trajectory_actual'][i]) // N_VECTORS
        ax.quiver(data['trajectory_actual'][i][::step, 0],
                  data['trajectory_actual'][i][::step, 1],
                  data['trajectory_actual'][i][::step, 2],
                  data['trajectory_target_points'][i][::step, 0]-data['trajectory_actual'][i][::step, 0],
                  data['trajectory_target_points'][i][::step, 1]-data['trajectory_actual'][i][::step, 1],
                  data['trajectory_target_points'][i][::step, 2]-data['trajectory_actual'][i][::step, 2],
                  color='red')
        print("Trajectory lag:", data['trajectory_lag'][i])
print(len(data['x']))

ax.set_title('Trajectory')
ax.set_xlabel('x, m')
ax.set_ylabel('y, m')
ax.set_zlabel('z, m')

ax.scatter(data['x'][0:1], data['y'][0:1], data['z'][0:1], color='green')
ax.text(data['x'][0], data['y'][0], data['z'][0]+0.4, s='Start point')
ax.scatter(data['x'][-1:], data['y'][-1:], data['z'][-1:], color='red')
ax.text(data['x'][-1], data['y'][-1], data['z'][-1]+0.4, s='Goal point', ha='right')

ax.scatter(data['pointcloud'][POINTCLOUD_IDX][::2, 0], data['pointcloud'][POINTCLOUD_IDX][::2, 1], data['pointcloud'][POINTCLOUD_IDX][::2, 2])
ax.set_xlim(0, 10)
ax.set_ylim(-5, 5)

plt.show()
