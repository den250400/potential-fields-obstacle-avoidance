import pickle
from matplotlib import pyplot as plt


with open('../logs/flight.pickle', 'rb') as file:
    data = pickle.load(file)

plt.plot(data['time'], data['v_pf'], label='x')
plt.plot(data['time'], data['vy'], label='y')
plt.plot(data['time'], data['vz'], label='z')
plt.title('X, Y, Z координаты')
plt.xlabel('Время с начала миссии, с')
plt.ylabel('Координата, м')
plt.legend()
plt.show()

fig = plt.figure(figsize=(10, 10))
ax = fig.gca(projection='3d')
ax.plot(data['x'], data['y'], data['z'])
print(len(data['x']))

ax.set_title('Траектория полета')
ax.set_xlabel('x, м')
ax.set_ylabel('y, м')
ax.set_zlabel('z, м')

ax.scatter(data['x'][0:1], data['y'][0:1], data['z'][0:1], color='green')
ax.text(data['x'][0], data['y'][0], data['z'][0], s='Точка взлета')
ax.scatter(data['x'][-1:], data['y'][-1:], data['z'][-1:], color='red')
ax.text(data['x'][-1], data['y'][-1], data['z'][-1], s='Точка посадки', ha='right')

plt.show()
