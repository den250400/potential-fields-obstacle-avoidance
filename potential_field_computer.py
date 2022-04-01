import numpy as np


class PotentialFieldComputer:
    def __init__(self, q_obstacle, q_target, q_copter, dist_threshold):
        self.q_obstacle = q_obstacle
        self.q_target = q_target
        self.q_copter = q_copter
        self.dist_threshold = dist_threshold

    def compute_trajectory(self, point_cloud, vehicle_coord, target_coord, speed, dt=0.1, n_points=20):
        trajectory = np.zeros(shape=(n_points, 3))
        trajectory[0] = vehicle_coord
        for i in range(1, n_points):
            if len(point_cloud) == 0:
                repelling_force = np.zeros(3)
            else:
                diff = point_cloud - vehicle_coord.reshape(1, 3)
                dist = np.linalg.norm(diff, axis=1, keepdims=True)
                repel_directions = -diff / dist

                distance_component = (1 / dist - 1 / self.dist_threshold)

                nullifier = np.ones_like(diff)
                nullifier[dist.reshape(-1) > self.dist_threshold, :] = np.zeros(3)

                repelling_force = np.sum(repel_directions * self.q_copter * self.q_obstacle * nullifier * distance_component, axis=0)

            attraction_force = 2 * self.q_copter * self.q_target * (target_coord - vehicle_coord) / np.linalg.norm(target_coord - vehicle_coord)

            force = attraction_force + repelling_force
            velocity = speed * force / np.linalg.norm(force)
            vehicle_coord = vehicle_coord + velocity * dt

            trajectory[i] = vehicle_coord

        return trajectory.copy()
