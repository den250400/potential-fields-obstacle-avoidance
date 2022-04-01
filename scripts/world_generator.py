from xml.etree import ElementTree
from xml.dom import minidom
import numpy as np

# Paths
BASE_WORLD_PATH = '/home/denis/catkin_ws/src/clover/clover_simulation/resources/worlds/clover.world'
OBSTACLE_WORLD_PATH = '/home/denis/catkin_ws/src/clover/clover_simulation/resources/worlds/random_obstacles.world'
# Obstacle generation parameters
OBSTACLE_PATH = 'model://pine_tree'
OBSTACLE_NAME = 'pine_tree'
REGION_XLIM = [5, 25]
REGION_YLIM = [-10, 10]
N_OBSTACLES = 20

# Open base world file and find the 'world' element
et = ElementTree.parse(BASE_WORLD_PATH)
world = et.find('world')

# Generate random obstacles' coordinates
x = np.random.uniform(REGION_XLIM[0], REGION_XLIM[1], size=N_OBSTACLES)
y = np.random.uniform(REGION_YLIM[0], REGION_YLIM[1], size=N_OBSTACLES)
print(x, y)

# Add obstacles to the world
insert_idx = 2
for i in range(N_OBSTACLES):
    obstacle = ElementTree.Element('include')
    uri = ElementTree.SubElement(obstacle, 'uri')
    uri.text = OBSTACLE_PATH
    name = ElementTree.SubElement(obstacle, 'name')
    name.text = OBSTACLE_NAME + '_' + str(i)
    pose = ElementTree.SubElement(obstacle, 'pose')
    pose.text = "%.2f %.2f %.2f %.2f %.2f %.2f" % (x[i], y[i], 0, 0, 0, 0)
    world.insert(i + insert_idx, obstacle)

xmlstr = minidom.parseString(ElementTree.tostring(et.getroot())).toprettyxml(indent="   ")
with open(OBSTACLE_WORLD_PATH, "w") as f:
    f.write(xmlstr)