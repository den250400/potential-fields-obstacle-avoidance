import rospy
import numpy as np
from clover import srv
from std_srvs.srv import Trigger
import math


TAKEOFF_ALT = 2
FLIGHT_DIST = 4
SPEED = 1
TRAJECTORY_N_POINTS = 60

set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='map', tolerance=0.2, auto_arm=True):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)


def _fly_trajectory_v(trajectory, speed, lead=2, tolerance=0.5):
    # Pre-compute initial lead index
    telem = get_telemetry(frame_id='map')
    diff = trajectory - np.array([telem.x, telem.y, telem.z]).reshape(1, 3)
    dist = np.linalg.norm(diff, axis=1)
    i = np.argmin(dist)
    while i < len(trajectory):
        # Move target point to fulfill lead requirement
        telem = get_telemetry(frame_id='map')
        while True:
            diff = trajectory[i] - np.array([telem.x, telem.y, telem.z])
            dist = np.linalg.norm(diff)

            if dist > lead or i == len(trajectory) - 1:
                break
            else:
                i += 1

        end_trajectory_pointer = trajectory[-1] - np.array([telem.x, telem.y, telem.z])

        yaw = math.atan2(end_trajectory_pointer[1], end_trajectory_pointer[0])
        vx = speed * diff[0] / dist
        vy = speed * diff[1] / dist
        vz = speed * diff[2] / dist

        set_velocity(vx=vx, vy=vy, vz=vz, yaw=yaw, frame_id='map')

        if math.sqrt((telem.x - trajectory[-1][0]) ** 2 + (telem.y - trajectory[-1][1]) ** 2 + (telem.z - trajectory[-1][2]) ** 2) < tolerance:
            break


telem = get_telemetry(frame_id='map')
x_target = telem.x + FLIGHT_DIST * math.cos(telem.yaw)
y_target = telem.y + FLIGHT_DIST * math.sin(telem.yaw)
z_target = telem.z + TAKEOFF_ALT

telem = get_telemetry(frame_id='map')
navigate_wait(x=telem.x, y=telem.y, z=z_target, yaw=telem.yaw, speed=SPEED)
trajectory_x = np.linspace(telem.x, x_target, TRAJECTORY_N_POINTS)
trajectory_y = np.linspace(telem.y, y_target, TRAJECTORY_N_POINTS)
trajectory_z = np.linspace(z_target, z_target, TRAJECTORY_N_POINTS)
trajectory = np.append(trajectory_x.reshape(-1, 1), trajectory_y.reshape(-1, 1), axis=1)
trajectory = np.append(trajectory, trajectory_z.reshape(-1, 1), axis=1)

_fly_trajectory_v(trajectory=trajectory, speed=SPEED)

land()
