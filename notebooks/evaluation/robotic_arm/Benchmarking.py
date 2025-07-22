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

def evaluate(configs: List[Dict]):
    for i, ts in enumerate([ts3, ts6, ts9]):
        for benchmark in ts.benchList:
            print(benchmark.name)
            for config in configs:
                benchmark: Benchmark

                config["dof"] = (i + 1) * 3
                config["lowLimits"] = [-2 *np.pi for _ in range(config["dof"])]
                config["highLimits"] = [2 *np.pi for _ in range(config["dof"])]
                config["discretization"] = [config["discretizationValue"] for _ in range(config["dof"])]

                astar = AStar(benchmark.collisionChecker)

                stats = {}
                title = benchmark.name
                
                start = time.time()
                solution, _ = astar.planPath(benchmark.startList, benchmark.goalList, config, store_viz=False)
                stats["execution_time"] = time.time() - start

                stats["road_map_size"] = len(astar.graph.nodes.keys())

                stats["num_nodes_solution_path"] = -1

                stats["solution_path_length"] = -1
                
                if solution != []:
                    stats["num_nodes_solution_path"] = len(solution)
                    
                    stats["solution_path_length"] = astar.graph.nodes[solution[-1]]["g"]

                    title += " (No path found!)"

                # save stats
                dir_name = f"{script_dir}/results/{3 * (i + 1)}DoF/{benchmark.name}"
                dir_name = f"{dir_name}/disc{config['discretization']}_{config['heuristic']}_w{config['w']}"
                if config["reopen"]:
                    dir_name += "_reopen"
                if config["check_connection"]:
                    dir_name += "_linetest"
                    if config["lazy_check_connection"]:
                        dir_name += "_lazy"
                    

                if not os.path.exists(dir_name):
                    os.makedirs(dir_name)

                stats_filename = f"{dir_name}/stats.json"
                with open(stats_filename, "w") as f:
                    json.dump(stats, f)
                
                graph_filename = f"{dir_name}/graph.json"
                with open(graph_filename, "w") as f:
                    json.dump(nx.node_link_data(astar.graph), f)

                solution_filename = f"{dir_name}/solution.json"
                with open(solution_filename, "w") as f:
                    json.dump(solution, f)


if __name__ == "__main__":
    astarConfig = dict()
    astarConfig["w"] = .5
    astarConfig["heuristic"]  = "euclidean"
    astarConfig["reopen"] = True
    astarConfig["discretizationValue"] = 30
    astarConfig["check_connection"] = True
    astarConfig["lazy_check_connection"] = True

    configs = []
    configs.append(astarConfig)
    
    evaluate(configs=configs)

