import numpy as np
import rospy
import ros_numpy
from clover import srv
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation

import threading


class PointCloudMapper:
    def __init__(self, depth_topic, pointcloud_size=2000, pointcloud_size_max=10000, downsample_factor=2):
        self.depth_topic = depth_topic
        self.pointcloud_size = pointcloud_size
        self.pointcloud_size_max = pointcloud_size_max
        self.downsample_factor = downsample_factor

        self.depth_subscriber = rospy.Subscriber(self.depth_topic, PointCloud2, self._depth_callback)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        self.init_telem = None
        self.recent_telem = None
        self.recent_pointcloud_msg = None
        self.pointcloud = np.zeros(shape=(0, 3))

        self.mapping_thread = threading.Thread(target=self._loop)

    def __del__(self):
        if self.mapping_thread.is_alive():
            self.mapping_thread.join()

    @staticmethod
    def _rotate_pointcloud(point_cloud, angles_list, degrees=True):
        r_mat = Rotation.from_euler('xyz', angles_list, degrees=degrees).as_matrix()
        rotated_point_cloud = np.matmul(r_mat, point_cloud.reshape(-1, 3, 1)).reshape(-1, 3)

        return rotated_point_cloud

    def _depth_callback(self, data):
        self.recent_telem = self.get_telemetry(frame_id='map')
        self.recent_pointcloud_msg = data

    def _loop(self):
        while True:
            if self.recent_pointcloud_msg is None:
                continue

            point_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.recent_pointcloud_msg)
            n_points_initial = point_arr.shape[0]
            # Downsample current shot (we do not want pointcloud of size 400k, right?)
            if len(point_arr) > self.pointcloud_size:
                point_arr = point_arr[::n_points_initial // self.pointcloud_size]

            # Depth camera outputs point cloud with z-axis pointing forward, compensate it
            point_arr = self._rotate_pointcloud(point_arr, [-90, 0, -90])

            # Compensate copter's position and orientation
            telem = self.recent_telem
            point_arr = self._rotate_pointcloud(point_arr, [telem.roll, telem.pitch, telem.yaw], degrees=False)
            point_arr = point_arr + np.array([telem.x, telem.y, telem.z]).reshape(1, 3)

            self.pointcloud = np.append(self.pointcloud, point_arr, axis=0)

            # Downsample the resulting pointcloud if it exceeds maximum size
            if self.pointcloud.shape[0] > self.pointcloud_size_max:
                self.pointcloud = self.pointcloud[::self.downsample_factor]

            rospy.sleep(1)

    def get_pointcloud(self):
        if self.pointcloud.shape[0] == 0:
            print("Point cloud appears to be empty. Did you call start() method?")

        return self.pointcloud

    def start(self):
        self.init_telem = self.get_telemetry(frame_id='map')
        self.mapping_thread.start()
