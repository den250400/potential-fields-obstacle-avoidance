import numpy as np
import time

from obstacle_avoidance import AvoidanceNavigation


nav = AvoidanceNavigation(np.array([16, 0, 2]))
time.sleep(3)
nav.start(speed=2)
