import rospy
import ros_numpy
import time
import numpy as np
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot as plt

rospy.init_node('stereo_node')

point_cloud = rospy.wait_for_message('/realsense/depth/color/points', PointCloud2)

t1 = time.time()
point_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud)[::100]
t2 = time.time()
print(t2 - t1)

r_mat = Rotation.from_euler('xz', [-90, -90], degrees=True).as_matrix()

point_arr = np.matmul(r_mat, point_arr.reshape(-1, 3, 1)).reshape(-1, 3)

fig = plt.figure(figsize=(10, 10))
ax = fig.gca(projection='3d')
ax.scatter(point_arr[:, 0], point_arr[:, 1], point_arr[:, 2])
ax.scatter([0], [0], [0], color='red')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()