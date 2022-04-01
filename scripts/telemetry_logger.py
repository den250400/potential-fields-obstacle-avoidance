import rospy
import time
import pickle
import threading
from clover import srv


class TelemetryLogger:
    def __init__(self, frame_id='map', frequency=10, require_armed=False, save_path='../logs/telem_log', save_period=10):
        self.frame_id = frame_id
        self.frequency = frequency
        self.require_armed = require_armed
        self.save_path = save_path
        self.save_period = save_period
        self.stop_flag = False

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

        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.write_history_thread = threading.Thread(target=self._write_telemtry_history)

    def _write_telemtry_history(self):
        start_time = time.time()
        while True:
            telem = self.get_telemetry(frame_id='map')
            if self.stop_flag:
                break
            if self.require_armed and not telem.armed:
                continue

            self.telem_history['x'].append(telem.x)
            self.telem_history['y'].append(telem.y)
            self.telem_history['z'].append(telem.z)
            self.telem_history['vx'].append(telem.vx)
            self.telem_history['vy'].append(telem.vy)
            self.telem_history['vz'].append(telem.vz)
            self.telem_history['yaw'].append(telem.yaw)
            self.telem_history['time'].append(time.time() - start_time)

            if len(self.telem_history['time']) % self.save_period == 0:
                with open(self.save_path, 'wb') as file:
                    pickle.dump(self.telem_history, file)

            rospy.sleep(1 / self.frequency)

    def start(self):
        self.write_history_thread.start()

    def stop(self):
        self.stop_flag = True
        self.write_history_thread.join()


telem_logger = TelemetryLogger()
telem_logger.start()
rospy.sleep(40)
telem_logger.stop()
