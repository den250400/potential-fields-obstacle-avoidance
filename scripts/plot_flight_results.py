import pickle
import numpy as np
from matplotlib import pyplot as plt


with open('../logs/flight.pickle', 'rb') as file:
    data = pickle.load(file)

vx = np.array(data['vx']).reshape(-1, 1)
vy = np.array(data['vy']).reshape(-1, 1)
vz = np.array(data['vz']).reshape(-1, 1)

v = np.append(vx, vy, axis=1)
v = np.append(v, vz, axis=1)

plt.plot(data['time'], np.linalg.norm(v, axis=1), label='v')
plt.xlabel('Время с начала миссии, с')
plt.ylabel('Скорость, м/с')
plt.legend()
plt.show()

fig = plt.figure(figsize=(10, 10))
ax = fig.gca(projection='3d')
ax.plot(data['x'], data['y'], data['z'], color='green')
print(len(data['x']))

ax.set_title('Траектория полета')
ax.set_xlabel('x, м')
ax.set_ylabel('y, м')
ax.set_zlabel('z, м')

ax.scatter(data['x'][0:1], data['y'][0:1], data['z'][0:1], color='green')
ax.text(data['x'][0], data['y'][0], data['z'][0], s='Точка взлета')
ax.scatter(data['x'][-1:], data['y'][-1:], data['z'][-1:], color='red')
ax.text(data['x'][-1], data['y'][-1], data['z'][-1], s='Точка посадки', ha='right')

ax.scatter(data['pointcloud'][::2, 0], data['pointcloud'][::2, 1], data['pointcloud'][::2, 2])

plt.show()
