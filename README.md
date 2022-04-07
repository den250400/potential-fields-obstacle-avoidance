# Obstacle avoidance based on artificial potential fields method
This repository contains obstacle avoidance system for quadcopters with Raspberry Pi 4 onboard computer. The code in this repository is designed to work with [Clover Raspberry Pi image](https://clover.coex.tech/en/image.html) and [special PX4-based firmware](https://clover.coex.tech/en/firmware.html) modified for easier communication with Raspberry Pi.

Artificial potential fields method is based on considering quadcopter, obstacles and target point as electric-charged points. Quadcopter and obstacles have positive charge, and target point is assigned with negative charge. This results in quadcopter "attracting" itself to the target point, while being repelled by the same-signed charges of obstacles. Using this analogy, one can compute a safe, collision-free trajectory, which can be later executed by the vehicle.

It's obvious that you need some sort of geometrical information about the surrounding world if you want to avoid obstacles. This algorithm uses Intel Realsense D435 depth camera - it provides a 3D point cloud which can be easily used for potential fields computation.

## Installation
First of all, install the [Clover simulator](https://clover.coex.tech/en/simulation.html). 
