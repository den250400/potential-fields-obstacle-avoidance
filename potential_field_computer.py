import numpy as np
from scipy.spatial.transform import Rotation
from matplotlib import pyplot as plt


class PotentialFieldComputer:
    def __init__(self, q_obstacle, q_target, q_copter):
        self.q_obstacle = q_obstacle
        self.q_target = q_target
        self.q_copter = q_copter

    @staticmethod
    def _rotate_pointcloud(point_cloud):
        r_mat = Rotation.from_euler('xz', [-90, -90], degrees=True).as_matrix()
        rotated_point_cloud = np.matmul(r_mat, point_cloud.reshape(-1, 3, 1)).reshape(-1, 3)

        return rotated_point_cloud

    def compute(self, point_cloud, r_to_target):
        point_cloud = self._rotate_pointcloud(point_cloud)
        point_cloud = point_cloud[point_cloud[:, 2] > -1.5]
        if len(point_cloud) == 0:
            repelling_force = np.zeros(3)
        else:
            d = np.linalg.norm(point_cloud, axis=1, keepdims=True)
            d[d < 2] = 1000
            boundary_part = 1 / (d - 2)

            obstacle_dist = point_cloud / (np.power(np.linalg.norm(point_cloud, axis=1, keepdims=True), 1)) * boundary_part
            repelling_force = np.sum(-self.q_copter * self.q_obstacle / obstacle_dist, axis=0) / point_cloud.shape[0]

        attraction_force = 2 * self.q_copter * self.q_target * r_to_target / np.linalg.norm(r_to_target)

        #print(repelling_force + attraction_force, len(point_cloud), np.min(np.power(np.linalg.norm(point_cloud, axis=1, keepdims=True), 1)))

        return repelling_force + attraction_force

    def compute_trajectory(self, point_cloud, r_to_target, velocity):
        point_cloud = self._rotate_pointcloud(point_cloud)
        point_cloud = point_cloud[point_cloud[:, 2] > -1.5]
        if len(point_cloud) == 0:
            repelling_force = np.zeros(3)
        else:
            d = np.linalg.norm(point_cloud, axis=1, keepdims=True)
            d[d < 2] = 1000
            boundary_part = 1 / (d - 2)

            obstacle_dist = point_cloud / (
                np.power(np.linalg.norm(point_cloud, axis=1, keepdims=True), 1)) * boundary_part
            repelling_force = np.sum(-self.q_copter * self.q_obstacle / obstacle_dist, axis=0) / point_cloud.shape[0]

        attraction_force = 2 * self.q_copter * self.q_target * r_to_target / np.linalg.norm(r_to_target)

        # print(repelling_force + attraction_force, len(point_cloud), np.min(np.power(np.linalg.norm(point_cloud, axis=1, keepdims=True), 1)))

        return repelling_force + attraction_force

