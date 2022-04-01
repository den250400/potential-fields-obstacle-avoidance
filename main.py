import numpy as np
import time

from obstacle_avoidance import AvoidanceNavigation


nav = AvoidanceNavigation(np.array([24, 3, 6]))
time.sleep(3)
nav.start()
