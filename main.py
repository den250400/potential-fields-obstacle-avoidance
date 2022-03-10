import numpy as np
import time

from obstacle_avoidance import AvoidanceNavigation


nav = AvoidanceNavigation(np.array([22, 0, 3]))
time.sleep(3)
nav.start()
