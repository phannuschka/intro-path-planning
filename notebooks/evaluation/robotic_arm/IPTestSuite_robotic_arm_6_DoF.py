import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from IPBenchmark import Benchmark 
from IPEnvironment import CollisionChecker
from shapely.geometry import Point, Polygon, LineString
import random
import numpy as np

benchList = list()
import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from IPBenchmark import Benchmark
from IPEnvironment import CollisionChecker
from IPEnvironmentKin import KinChainCollisionChecker
from IPPlanarManipulator import PlanarRobot
from shapely.geometry import Point, Polygon, LineString
import random
import numpy as np


n_joints = 6
robot = PlanarRobot(n_joints=n_joints, length=4.0)

benchList = list()

# -----------------------------------------

obst = dict()
obst["obs1"] = LineString([(-2, 0), (-0.8, 0)]).buffer(0.5)
obst["obs2"] = LineString([(2, 0), (2, 1)]).buffer(0.2)
obst["obs3"] = LineString([(-1, 2), (1, 2)]).buffer(0.1)

environment = KinChainCollisionChecker(robot, obst, limits=[[-3.14,3.14]] * n_joints, fk_resolution=.2)
start_joint_pos = [0.0] * n_joints
start_joint_pos[0] = 0.8
start_joint_pos = start_joint_pos[:n_joints]

end_joint_pos = [0.0] * n_joints
end_joint_pos[0] = 4
end_joint_pos = end_joint_pos[:n_joints]

description = "Planar manipulator has to find a path around simple obstacles."

benchmark = Benchmark("simple", environment, [start_joint_pos], [end_joint_pos], description, level=1)
benchList.append(benchmark)

# -----------------------------------------

obst = dict()
# Vertical wall with gap - creates narrow passage
obst["obs1"] = LineString([(-1, -4), (-1, 0.5)]).buffer(0.15)
obst["obs2"] = LineString([(-1, 1.2), (-1, 4)]).buffer(0.15)
# L-shaped obstacle in upper left
obst["obs3"] = LineString([(-3.5, 1), (-2, 1)]).buffer(0.2)
obst["obs4"] = LineString([(-3.5, 1), (-3.5, 3.5)]).buffer(0.2)

environment = KinChainCollisionChecker(robot, obst, limits=[[-3.14,3.14]] * n_joints, fk_resolution=.2)
start_joint_pos = [0.0] * n_joints
start_joint_pos[0] = 0.0
start_joint_pos = start_joint_pos[:n_joints]
end_joint_pos = [0.0] * n_joints
end_joint_pos[0] = 2.5
end_joint_pos = end_joint_pos[:n_joints]

description = "Planar manipulator has to find a path around simple obstacles with a narrow passage."

benchmark = Benchmark("narrow_passage", environment, [start_joint_pos], [end_joint_pos], description, level=2)
benchList.append(benchmark)

# -----------------------------------------

obst = dict()
# Layer 1: Outer ring with opening at 3 o'clock
obst["obs1"] = LineString([(0.75, -1.875), (-1.125, -1.5)]).buffer(0.15)  # bottom-left arc
obst["obs2"] = LineString([(-1.125, -1.5), (-1.5, 0.375)]).buffer(0.15)   # left side
obst["obs3"] = LineString([(-1.5, 0.375), (-0.375, 1.125)]).buffer(0.15)    # top-left arc
# Layer 2: Middle ring with opening at 9 o'clock
obst["obs4"] = LineString([(1.875, 0.75), (0.75, 1.875)]).buffer(0.135)     # top-right arc
obst["obs5"] = LineString([(0.75, 1.875), (-0.75, 2.25)]).buffer(0.135)    # top side
obst["obs6"] = LineString([(1.875, -0.375), (1.875, 0.75)]).buffer(0.135)    # right side
# Layer 3: Upper ring with opening at 6 o'clock
obst["obs7"] = LineString([(-1.875, 3.0), (-0.75, 3.75)]).buffer(0.12)   # top-left
obst["obs8"] = LineString([(-0.75, 3.75), (1.125, 3.6)]).buffer(0.12)    # top side
obst["obs9"] = LineString([(1.125, 3.6), (1.5, 2.4)]).buffer(0.12)     # right side
# "Trap" branches - dead ends that look promising
obst["obs10"] = LineString([(2.625, -0.75), (3.375, 0.0)]).buffer(0.1125)   # false right path
obst["obs11"] = LineString([(-2.4, 1.5), (-3.0, 2.25)]).buffer(0.1125)  # false left path

environment = KinChainCollisionChecker(robot, obst, limits=[[-3.14,3.14]] * n_joints, fk_resolution=.2)
start_joint_pos = [0.0] * n_joints
start_joint_pos[0] = np.pi/2
start_joint_pos[2] = np.pi/2 - 0.3
start_joint_pos[4] = 3 * np.pi/2
end_joint_pos = [0.0] * n_joints
end_joint_pos[0] = 3 * np.pi/2 + 0.75
end_joint_pos[2] = 1.0
end_joint_pos[4] = 0.8

description = "Planar manipulator has to find a path through a complex scene with multiple layers and dead ends."

benchmark = Benchmark("complex_scene", environment, [start_joint_pos], [end_joint_pos], description, level=3)
benchList.append(benchmark)