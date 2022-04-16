import rospy
from clover import srv
import math


TAKEOFF_ALTITUDE = 2

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)


def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='map', tolerance=0.2, auto_arm=True):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)


telem = get_telemetry(frame_id='map')

print("Starting takeoff")
navigate_wait(x=0, y=telem.y, z=TAKEOFF_ALTITUDE, yaw=0, speed=2)
print("Takeoff finished")
