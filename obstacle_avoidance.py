import rospy
import ros_numpy
import numpy as np
from clover import srv
from sensor_msgs.msg import PointCloud2
from potential_field_computer import PotentialFieldComputer

import math
import time
import pickle
import threading


rospy.init_node('obstacle_avoidance')


class AvoidanceNavigation:
    def __init__(self, target_point, depth_topic='/realsense/depth/color/points', target_point_frame='map',
                 frequency=0.01, point_cloud_size=3000, target_point_tolerance=0.3):
        self.target_point = target_point
        self.target_point_frame = target_point_frame
        self.pf_computer = PotentialFieldComputer(q_obstacle=1, q_target=3, q_copter=1)
        self.frequency = frequency
        self.point_cloud_size = point_cloud_size
        self.depth_topic = depth_topic
        self.target_point_tolerance = target_point_tolerance

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
            'v_pf': [],
            'time': []
        }

        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        self.write_history_thread = threading.Thread(target=self._write_telemtry_history)
        self.write_history_thread.start()

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
            self.telem_history['v_pf'].append(self.v_pf)
            self.telem_history['time'].append(time.time() - start_time)
            rospy.sleep(self.frequency)

    def start(self):
        print("Starting the obstacle avoidance flight")
        self.flying = True
        while True:
            telem_map = self.get_telemetry(frame_id='map')
            if self._check_finish_condition(telem_map):
                break
            point_cloud = rospy.wait_for_message(self.depth_topic, PointCloud2)

            point_arr = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud)
            n_points_initial = point_arr.shape[0]
            if len(point_arr) < self.point_cloud_size:
                point_arr = np.zeros((0, 3))
            else:
                point_arr = point_arr[::n_points_initial // self.point_cloud_size]

            r_to_target = np.array([self.target_point[0] - telem_map.x, self.target_point[1] - telem_map.y, self.target_point[2] - telem_map.z])
            desired_velocity = self.pf_computer.compute(point_arr, r_to_target)
            self.v_pf = np.linalg.norm(desired_velocity)

            desired_velocity = 1.5 * desired_velocity / np.linalg.norm(desired_velocity)

            target_yaw = math.atan2(desired_velocity[1], desired_velocity[0])

            self.set_velocity(vx=desired_velocity[0], vy=desired_velocity[1], vz=desired_velocity[2], frame_id='body')

        self.flying = False
        self.navigate(x=self.target_point[0], y=self.target_point[1], z=self.target_point[2], yaw=0, speed=1, frame_id='map')
        print("Target point achieved, disengaging obstacle avoidance mode")

        with open('./logs/flight.pickle', 'wb') as file:
            pickle.dump(self.telem_history, file)
