import rospy
import numpy as np
from clover import srv
from matplotlib import pyplot as plt
from potential_field_computer import PotentialFieldComputer
from mapping import PointCloudMapper

import math
import time
import pickle
import threading


rospy.init_node('obstacle_avoidance')


class AvoidanceNavigation:
    def __init__(self, target_point, depth_topic='/realsense/depth/color/points', target_point_frame='map',
                 frequency=0.01, point_cloud_size=3000, target_point_tolerance=0.7, trajectory_tolerance=0.3):
        self.target_point = target_point
        self.target_point_frame = target_point_frame
        self.pf_computer = PotentialFieldComputer(q_obstacle=1, q_target=20, q_copter=1, dist_threshold=2)
        self.mapper = PointCloudMapper(depth_topic)
        self.frequency = frequency
        self.point_cloud_size = point_cloud_size
        self.depth_topic = depth_topic
        self.target_point_tolerance = target_point_tolerance
        self.trajectory_tolerance = trajectory_tolerance

        self.flying = False
        self.v_pf = 0
        self.telem_history = {
            'x': [],
            'y': [],
            'z': [],
            'vx': [],
            'vy': [],
            'vz': [],
            'yaw': [],
            'time': []
        }

        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        self.write_history_thread = threading.Thread(target=self._write_telemtry_history)
        self.write_history_thread.start()

        self.mapper.start()

    def _check_finish_condition(self, telem_map):
        print(math.sqrt((telem_map.x - self.target_point[0]) ** 2 + (telem_map.y - self.target_point[1]) ** 2 +
                     (telem_map.z - self.target_point[2]) ** 2))
        if math.sqrt((telem_map.x - self.target_point[0]) ** 2 + (telem_map.y - self.target_point[1]) ** 2 +
                     (telem_map.z - self.target_point[2]) ** 2) < self.target_point_tolerance:
            return True
        else:
            return False

    def _write_telemtry_history(self):
        start_time = time.time()
        while True:
            if not self.flying:
                continue

            telem = self.get_telemetry(frame_id='map')
            self.telem_history['x'].append(telem.x)
            self.telem_history['y'].append(telem.y)
            self.telem_history['z'].append(telem.z)
            self.telem_history['vx'].append(telem.vx)
            self.telem_history['vy'].append(telem.vy)
            self.telem_history['vz'].append(telem.vz)
            self.telem_history['yaw'].append(telem.yaw)
            self.telem_history['time'].append(time.time() - start_time)
            rospy.sleep(self.frequency)

    def _fly_trajectory(self, trajectory, speed):
        tl = self.get_telemetry(frame_id='map')
        """
        r_mat = Rotation.from_euler('xyz', [-tl.roll, -tl.pitch, -tl.yaw], degrees=True).as_matrix()
        trajectory = np.matmul(r_mat, trajectory.reshape(-1, 3, 1)).reshape(-1, 3)
        trajectory = trajectory + np.array([tl.x, tl.y, tl.z]).reshape(1, 3)
        """

        traj_length = np.sum(np.linalg.norm(trajectory[1:] - trajectory[:-1], axis=1))
        dt = traj_length / (speed * (trajectory.shape[0] - 1))

        for i in range(1, len(trajectory)):
            tl = self.get_telemetry(frame_id='map')
            yaw = math.atan2(trajectory[i, 1] - tl.y, trajectory[i, 0] - tl.x)
            self.set_position(x=trajectory[i, 0], y=trajectory[i, 1], z=trajectory[i, 2], yaw=yaw, frame_id='map')
            rospy.sleep(dt+0.001)

    def start(self):
        print("Starting the obstacle avoidance flight")
        self.flying = True
        while True:
            telem_map = self.get_telemetry(frame_id='map')
            if self._check_finish_condition(telem_map):
                break

            pointcloud = self.mapper.get_pointcloud()
            if pointcloud.shape[0] == 0:
                continue

            vehicle_coord = np.array([telem_map.x, telem_map.y, telem_map.z])
            local_trajectory = self.pf_computer.compute_trajectory(point_cloud=pointcloud,
                                                                   vehicle_coord=vehicle_coord,
                                                                   target_coord=self.target_point,
                                                                   speed=3,
                                                                   dt=0.05,
                                                                   n_points=60)

            self._fly_trajectory(local_trajectory, 5)

        self.flying = False
        self.navigate(x=self.target_point[0], y=self.target_point[1], z=self.target_point[2], yaw=0, speed=1, frame_id='map')
        print("Target point achieved, disengaging obstacle avoidance mode")

        with open('./logs/flight.pickle', 'wb') as file:
            pickle.dump(self.telem_history, file)