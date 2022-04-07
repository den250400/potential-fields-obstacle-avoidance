# Obstacle avoidance based on artificial potential fields method

This repository contains obstacle avoidance system for quadcopters with Raspberry Pi 4 onboard computer. The code in this repository is designed to work with [Clover Raspberry Pi image](https://clover.coex.tech/en/image.html) and [special PX4-based firmware](https://clover.coex.tech/en/firmware.html) modified for easier communication with Raspberry Pi.

Artificial potential fields method is based on considering quadcopter, obstacles and target point as electric-charged points. Quadcopter and obstacles have positive charge, and target point is assigned with negative charge. This results in quadcopter "attracting" itself to the target point, while being repelled by the same-signed charges of obstacles. Using this analogy, you can compute a safe, collision-free trajectory, which can be executed by the vehicle.

<p align="center">
  <img src="https://github.com/den250400/potential-fields-obstacle-avoidance/blob/main/assets/Traditional-artificial-potentials-path-planning_Q320.jpg" />
</p>

It's obvious that you need some sort of geometrical information about the surrounding world if you want to avoid obstacles. This algorithm uses Intel Realsense D435 depth camera - it provides a 3D point cloud which can be easily used for potential fields computation.

## Installation
This guide is intended to be used on Ubuntu 20.04

1. Install the [Clover simulator](https://clover.coex.tech/en/simulation.html). 
2. Install [realsense_gazebo_plugin](https://github.com/issaiass/realsense_gazebo_plugin) and [realsense2_description](https://github.com/issaiass/realsense2_description):
```bash
cd ~/catkin_ws/src
git clone https://github.com/issaiass/realsense_gazebo_plugin
git clone https://github.com/issaiass/realsense2_description
cd ~/catkin_ws
catkin_make
```
3. Replace your ~/catkin_ws/src/clover/clover_description folder with one in this repository. This will add Realsense D435 to quadcopter model and make other minor changes so you will be able to reproduce our results.

