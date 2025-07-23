import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

import IPTestSuite_robotic_arm_3_DoF as ts3
import IPTestSuite_robotic_arm_6_DoF as ts6
import IPTestSuite_robotic_arm_9_DoF as ts9
from IPEnvironment import CollisionChecker
from IPBenchmark import Benchmark
from IPLazyPRM import LazyPRM
import matplotlib.pylab as plt
from shapely.geometry import Point, Polygon, LineString
from shapely import plotting
from IPAStar import AStar
from IPVISAStar import aStarVisualize
from IPPerfMonitor import IPPerfMonitor
import time
import json
import networkx as nx
from typing import List, Dict
import numpy as np

testSuits = {3: ts3, 6: ts6, 9: ts9}

def evaluate(configs: List[Dict], algorithm="astar"):
    for config in configs:
        ts = testSuits[config["dof"]]
        benchmarks = [ts.benchList[i] for i in config["benchmarks"]]
        for benchmark in benchmarks:
            print(benchmark.name)

            stats = {}
            title = benchmark.name

            match algorithm:
                case "astar":
                    solver = AStar(benchmark.collisionChecker)

                    start = time.time()
                    solution, _ = solver.planPath(benchmark.startList, benchmark.goalList, config, store_viz=False)
                    stats["execution_time"] = time.time() - start

                    dir_name = f"{script_dir}/results/{config['dof']}DoF/{benchmark.name}"
                    dir_name = f"{dir_name}/disc{config['discretization']}_{config['heuristic']}_w{config['w']}"
                    if config["reopen"]:
                        dir_name += "_reopen"
                    if config["check_connection"]:
                        dir_name += "_linetest"
                        if config["lazy_check_connection"]:
                            dir_name += "_lazy"

                case "prm":
                    solver = LazyPRM(benchmark.collisionChecker)
                    lazyConfig = dict()
                    lazyConfig["initialRoadmapSize"] = 500
                    lazyConfig["updateRoadmapSize"]  = 50
                    lazyConfig["kNearest"] = 20
                    lazyConfig["maxIterations"] = 2000

                    startList = benchmark.startList
                    goalList  = benchmark.goalList

                    start = time.time()
                    solution = solver.planPath(startList, goalList, lazyConfig)
                    stats["execution_time"] = time.time() - start

                    dir_name = f"{script_dir}/results/{config['dof']}DoF/{benchmark.name}"
                    dir_name = f"{dir_name}/PRM"

                case _:
                    continue

            stats["road_map_size"] = len(solver.graph.nodes.keys())

            stats["num_nodes_solution_path"] = -1

            stats["solution_path_length"] = -1

            if solution != []:
                stats["num_nodes_solution_path"] = len(solution)

                match algorithm:
                    case "astar":
                        stats["solution_path_length"] = solver.graph.nodes[solution[-1]]["g"]

                    case "prm":
                        length = 0
                        pos1 = np.array(solver.graph.nodes[solution[0]]["pos"])
                        for node in solution[1:]:
                            pos2 = np.array(solver.graph.nodes[node]["pos"])
                            length += np.linalg.norm(pos2 -pos1)

                        stats["solution_path_length"] = length

                title += " (No path found!)"

            if not os.path.exists(dir_name):
                os.makedirs(dir_name)

            stats_filename = f"{dir_name}/stats.json"
            with open(stats_filename, "w") as f:
                json.dump(stats, f)
            
            graph_filename = f"{dir_name}/graph.json"
            with open(graph_filename, "w") as f:
                json.dump(nx.node_link_data(solver.graph), f)

            solution_filename = f"{dir_name}/solution.json"
            with open(solution_filename, "w") as f:
                json.dump(solution, f)


if __name__ == "__main__":
    configs = []
    for disc in [4, 6]:
        for i in range(1, 4):
            disc_config = dict()
            disc_config["dof"] = 3 * i
            disc_config["lowLimits"] = [-2 *np.pi for _ in range(disc_config["dof"])]
            disc_config["highLimits"] = [2 *np.pi for _ in range(disc_config["dof"])]
            disc_config["discretization"] = [disc for _ in range(disc_config["dof"])]
            disc_config["w"] = .5
            disc_config["heuristic"]  = "euclidean"
            disc_config["reopen"] = True
            disc_config["check_connection"] = True
            disc_config["lazy_check_connection"] = True
            disc_config["benchmarks"] = [0, 1, 2]

            configs.append(disc_config)

    for disc in [10, 20]:
        for i in range(1, 3):
            disc_config = dict()
            disc_config["dof"] = 3 * i
            disc_config["lowLimits"] = [-2 *np.pi for _ in range(disc_config["dof"])]
            disc_config["highLimits"] = [2 *np.pi for _ in range(disc_config["dof"])]
            disc_config["discretization"] = [disc for _ in range(disc_config["dof"])]
            disc_config["w"] = .5
            disc_config["heuristic"]  = "euclidean"
            disc_config["reopen"] = True
            disc_config["check_connection"] = True
            disc_config["lazy_check_connection"] = True
            disc_config["benchmarks"] = [0, 1, 2]

            configs.append(disc_config)

    disc_config = dict()
    disc_config["dof"] = 3
    disc_config["lowLimits"] = [-2 *np.pi for _ in range(disc_config["dof"])]
    disc_config["highLimits"] = [2 *np.pi for _ in range(disc_config["dof"])]
    disc_config["discretization"] = [50 for _ in range(disc_config["dof"])]
    disc_config["w"] = .5
    disc_config["heuristic"]  = "euclidean"
    disc_config["reopen"] = True
    disc_config["check_connection"] = True
    disc_config["lazy_check_connection"] = True
    disc_config["benchmarks"] = [0, 1, 2]

    configs.append(disc_config)

    for w in [0.5, 0.75, 1]:
        w_config  = dict()
        w_config["dof"] = 3
        w_config["lowLimits"] = [-2 *np.pi for _ in range(w_config["dof"])]
        w_config["highLimits"] = [2 *np.pi for _ in range(w_config["dof"])]
        w_config["discretization"] = [50 for _ in range(w_config["dof"])]
        w_config["w"] = w
        w_config["heuristic"]  = "euclidean"
        w_config["reopen"] = True
        w_config["check_connection"] = True
        w_config["lazy_check_connection"] = True
        w_config["benchmarks"] = [0, 1, 2]

        configs.append(w_config)

    reopen_config  = dict()
    reopen_config["dof"] = 3
    reopen_config["lowLimits"] = [-2 *np.pi for _ in range(reopen_config["dof"])]
    reopen_config["highLimits"] = [2 *np.pi for _ in range(reopen_config["dof"])]
    reopen_config["discretization"] = [50 for _ in range(reopen_config["dof"])]
    reopen_config["w"] = 0.75
    reopen_config["heuristic"]  = "euclidean"
    reopen_config["reopen"] = False
    reopen_config["check_connection"] = True
    reopen_config["lazy_check_connection"] = True
    reopen_config["benchmarks"] = [0, 1, 2]

    configs.append(reopen_config)
    
    
    evaluate(configs=configs, algorithm="astar")

