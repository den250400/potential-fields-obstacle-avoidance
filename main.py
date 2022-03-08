import numpy as np
import time

from obstacle_avoidance import AvoidanceNavigation


time.sleep(2)
nav = AvoidanceNavigation(np.array([9, 0, 3]))
nav.start()
