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


n_joints = 3
robot = PlanarRobot(n_joints=n_joints, length=4.0)

benchList = list()

# -----------------------------------------

obst = dict()
obst["obs1"] = LineString([(-1, -4), (-1, 0.5)]).buffer(0.15)
obst["obs2"] = LineString([(-1, 1.2), (-1, 4)]).buffer(0.15)
obst["obs3"] = LineString([(-3.5, 1), (-2, 1)]).buffer(0.2)
obst["obs4"] = LineString([(-3.5, 1), (-3.5, 3.5)]).buffer(0.2)

environment = KinChainCollisionChecker(robot, obst, limits=[[-3.14,3.14]] * n_joints, fk_resolution=.2)
start_joint_pos = [0.0] * 9
start_joint_pos[0] = 0.0
start_joint_pos = start_joint_pos[:n_joints]

end_joint_pos = [0.0] * 9
end_joint_pos[0] = 2.5
end_joint_pos = end_joint_pos[:n_joints]

description = "Planar manipulator with 3 DoF has to find a path around simple obstacles."

benchmark = Benchmark("simple", environment, [start_joint_pos], [end_joint_pos], description, level=1)
benchList.append(benchmark)

# -----------------------------------------
