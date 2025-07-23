# coding: utf-8
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

# -----------------------------------------

num_bubbles = 40
min_size = 0.1
max_size = 2
borders_low = [0, 0]
borders_high = [20, 20]
random_seed = "12ABQr"
random.seed(random_seed)
start = [2, 2]
goal = [18, 18]
bubble_bath = dict()
for i in range(num_bubbles):
    coord_x = random.uniform(borders_low[0], borders_high[0])
    coord_y = random.uniform(borders_low[1], borders_high[1])
    size = random.uniform(min_size, max_size)

    coord = np.array([coord_x, coord_y])
    dist_start = np.linalg.norm(np.array(start) - coord)
    dist_goal = np.linalg.norm(np.array(goal) - coord)
    if (dist_start > size) and (dist_goal > size):
        bubble_bath["bubble_"+str(i)] = Point((coord_x, coord_y)).buffer(size)

description = "Planer has to find a passage around the bubbles"
benchList.append(Benchmark("Bubble_Bath", CollisionChecker(bubble_bath), [start], [goal], description, 2))

# -----------------------------------------

num_sticks = 40
borders_low = [0, 0]
borders_high = [20, 20]
size = 4
random_seed = "12Abcb"
random.seed(random_seed)
start = [2, 2]
goal = [18, 18]
mikado = dict()
for i in range(num_sticks):
    coord_x1 = random.uniform(borders_low[0], borders_high[0])
    coord_y1 = random.uniform(borders_low[1], borders_high[1])

    direction = np.array([random.uniform(-1, 1), random.uniform(-1, 1)])
    direction = direction / np.linalg.norm(direction)

    mikado["stick_"+str(i)] = LineString([(coord_x1, coord_y1), (coord_x1 + direction[0] * size, coord_y1 + direction[1] * size)]).buffer(0.2)

description = "Planer has to find a passage around the mikado sticks"
benchList.append(Benchmark("Mikado", CollisionChecker(mikado), [start], [goal], description, 2))

# -----------------------------------------

start = [2, 12]
goal = [2, 8]

line_collision_test = dict()
line_collision_test["trjiangle_1"] = Polygon([(-20,  11.5), (-20,  10.5), (18, 11)])
line_collision_test["trjiangle_2"] = Polygon([(40,  9.5), (40,  8.5), (2, 9)])


description = "Planer has to find a passage around very narrow obstacles challenging the line collision test"
benchList.append(Benchmark("Line_Collision_Test", CollisionChecker(line_collision_test), [start], [goal], description, 3))

# -----------------------------------------

gap_sizes = [0.1 * (1.4**i) for i in range(20)]
segment_length = 1
segment_width = 4
gap_offset = 0.3
segment_start = 2

start = [2, 18]
goal = [2, 2]

gaps = dict()
gaps["segment_0"] = Polygon([(-20,  10 + (segment_width / 2)), (-20,  10 - (segment_width / 2)), (segment_start + gap_offset, 10 - (segment_width / 2)), (segment_start, 10 + (segment_width / 2))])

for i, gap_size in enumerate(gap_sizes):
    segment_start += gap_size
    gaps["segment_"+str(i + 1)] = Polygon([(segment_start,  10 + (segment_width / 2)), (segment_start + gap_offset,  10 - (segment_width / 2)), (segment_start + gap_offset + segment_length, 10 - (segment_width / 2)), (segment_start + segment_length, 10 + (segment_width / 2))])
    segment_start += segment_length

description = "Planer has to find a passage through gaps with smaller gaps allowing for shorter paths"
benchList.append(Benchmark("Gaps", CollisionChecker(gaps), [start], [goal], description, 4))

# -----------------------------------------

layers = 9
offset = layers + 3
maze = dict()
for i in range(1, layers):
    if i % 2 == 1:
        maze["ring_wall"+str(i)] = LineString([(offset + 1, offset - i),
                                               (offset + i, offset - i),
                                               (offset + i, offset + i),
                                               (offset - i, offset + i),
                                               (offset - i, offset - i),
                                               (offset - 1, offset - i)]).buffer(0.2)
        if i > 1:
            maze["divider"+str(i)] = LineString([(offset + i, offset), (offset + i - 1, offset)]).buffer(0.2)
    else:
        maze["ring_wall"+str(i)] = LineString([(offset + 1, offset + i),
                                               (offset + i, offset + i),
                                               (offset + i, offset - i),
                                               (offset - i, offset - i),
                                               (offset - i, offset + i),
                                               (offset - 1, offset + i)]).buffer(0.2)
        maze["divider"+str(i)] = LineString([(offset - i, offset), (offset + 1 - i, offset)]).buffer(0.2)
description = "Planer has to find a passage out of the maze"
benchList.append(Benchmark("Maze", CollisionChecker(maze), [[offset, offset]], [[offset, offset + layers]], description, 5))

# -----------------------------------------

resolution = 100
spiral_radius = 8
num_turns = 4
num_iterations = resolution * num_turns
offset = spiral_radius + 2
spiral = dict()
spiral_corners_1 = []
spiral_corners_2 = []
for i in range(0, num_iterations + 1):
    corner_x = spiral_radius * np.sin(2 * np.pi * (i / resolution)) * (i / num_iterations)
    corner_y = spiral_radius * np.cos(2 * np.pi * (i / resolution)) * (i / num_iterations)
    spiral_corners_1.append((offset + corner_x, offset + corner_y))
    spiral_corners_2.append((offset - corner_x, offset - corner_y))
spiral["spiral_1"] = LineString(spiral_corners_1).buffer(0.2)
spiral["spiral_2"] = LineString(spiral_corners_2).buffer(0.2)
description = "Planer has to find a passage out of the spiral and back in the other way"
benchList.append(Benchmark("Spiral", CollisionChecker(spiral), [[offset + 1, offset]], [[offset - 1, offset]], description, 5))