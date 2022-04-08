import rospy
import numpy as np
from clover import srv
from potential_field_computer import PotentialFieldComputer
from mapping import PointCloudMapper

import math
import time
import pickle
import threading

rospy.init_node('obstacle_avoidance')


class AvoidanceNavigation:
    def __init__(self, target_point, pointcloud_topic='/camera/depth/color/points', target_point_frame='map',
                 target_point_tolerance=1, tl_write_frequency=0.01, traj_compute_frequency=0.1,
                 map_update_frequency=0.2):
        """

        :param target_point: point where the vehicle needs to go while avoiding obstacles
        :param pointcloud_topic: name of the topic with point cloud
        :param target_point_frame: frame_id of target point
        :param target_point_tolerance: when the distance (in meters) to target point is less
        than this, obstacle avoidance mode is disengaged, and copter starts holding the target point
        :param tl_write_frequency: wait period before writing next telemetry to history array
        :param traj_compute_frequency: wait period before generating next local trajectory
        :param map_update_frequency: wait period before updating the pointcloud map with the new reading from
        pointcloud_topic
        """
        self.target_point = target_point
        self.target_point_frame = target_point_frame
        self.apf_computer = PotentialFieldComputer(q_repel=1, q_attract=20, dist_threshold=2)
        self.mapper = PointCloudMapper(pointcloud_topic, update_frequency=map_update_frequency)
        self.tl_write_frequency = tl_write_frequency
        self.traj_compute_frequency = traj_compute_frequency
        self.pointcloud_topic = pointcloud_topic
        self.target_point_tolerance = target_point_tolerance

        self.flying = False
        self.v_pf = 0
        self.telem = None
        self.telem_history = {
            'x': [],
            'y': [],
            'z': [],
            'vx': [],
            'vy': [],
            'vz': [],
            'yaw': [],
            'pointcloud': [],
            'local_trajectories': [],
            'time': []
        }

        # Vehicle control services
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
        self.set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

        # Initializing auxiliary threads
        self.write_history_thread = threading.Thread(target=self._write_telemtry_history)
        self.telem_update_thread = threading.Thread(target=self._telem_updater)
        self.traj_control_thread = None
        self.write_history_thread.start()
        self.telem_update_thread.start()

        self.mapper.start()

        self.debug_written = False
        self.update_traj_flag = False

    def _check_finish_condition(self, telem_map):
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
            rospy.sleep(self.tl_write_frequency)

    def _telem_updater(self):
        while True:
            self.telem = self.get_telemetry(frame_id='map')

    def _fly_trajectory_v(self, trajectory, speed, lead=0.8):
        self.update_traj_flag = False  # Reset the trajectory execution stopper flag

        # Pre-compute initial lead index
        diff = trajectory - np.array([self.telem.x, self.telem.y, self.telem.z]).reshape(1, 3)
        dist = np.linalg.norm(diff, axis=1)
        i = np.argmin(dist)
        while i < len(trajectory):
            if not self.flying or self.update_traj_flag:
                break

            # Move target point to fulfill lead requirement
            while True:
                diff = trajectory[i] - np.array([self.telem.x, self.telem.y, self.telem.z])
                dist = np.linalg.norm(diff)

                if dist > lead or i == len(trajectory) - 1:
                    break
                else:
                    i += 1

            end_trajectory_pointer = trajectory[-1] - np.array([self.telem.x, self.telem.y, self.telem.z])

            yaw = math.atan2(end_trajectory_pointer[1], end_trajectory_pointer[0])
            vx = speed * diff[0] / dist
            vy = speed * diff[1] / dist
            vz = speed * diff[2] / dist
            self.set_velocity(vx=vx, vy=vy, vz=vz, yaw=yaw, frame_id='map')

    def _update_trajectory(self, trajectory, speed):
        if self.traj_control_thread is not None:
            self.update_traj_flag = True  # This flag will be read in current self._fly_trajectory thread
            self.traj_control_thread.join()

        self.traj_control_thread = threading.Thread(target=self._fly_trajectory_v, args=(trajectory, speed))
        self.traj_control_thread.start()

    def align_vehicle(self):
        # Point copter towards the target point
        telem = self.get_telemetry(frame_id='map')
        target_yaw = math.atan2(self.target_point[1] - telem.y, self.target_point[0] - telem.x)
        self.set_velocity(vx=0, vy=0, vz=0, yaw=target_yaw)
        while math.fabs(self.telem.yaw - target_yaw) > 0.1:
            continue

    def start(self, speed=2):
        print("Starting the obstacle avoidance flight")
        self.flying = True

        # Point copter towards the target point
        self.align_vehicle()
        rospy.sleep(2)

        while True:
            if self._check_finish_condition(self.telem):
                break

            # Get current point cloud
            pointcloud = self.mapper.get_pointcloud()
            if pointcloud.shape[0] == 0:
                continue

            vehicle_coord = np.array([self.telem.x, self.telem.y, self.telem.z])
            local_trajectory = self.apf_computer.compute_trajectory(point_cloud=pointcloud,
                                                                    vehicle_coord=vehicle_coord,
                                                                    target_coord=self.target_point,
                                                                    speed=speed,
                                                                    dt=0.05,
                                                                    n_points=60)

            self.telem_history['local_trajectories'].append(local_trajectory)
            self.telem_history['pointcloud'].append(self.mapper.get_pointcloud())
            self._update_trajectory(local_trajectory, speed)
            rospy.sleep(self.traj_compute_frequency)

        self.flying = False
        self.set_velocity(vx=0, vy=0, vz=0, frame_id='map')
        self.navigate(x=self.target_point[0], y=self.target_point[1], z=self.target_point[2], yaw=0, speed=1,
                      frame_id='map')
        print("Target point achieved, disengaging obstacle avoidance mode")

        with open('logs/flight.pickle', 'wb') as file:
            pickle.dump(self.telem_history, file)
