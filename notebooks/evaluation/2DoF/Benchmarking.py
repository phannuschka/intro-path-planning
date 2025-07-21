import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

import IPTestSuite_2DoF as ts
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

def evaluate(configs: List[Dict]):
    for benchmark in ts.benchList:
        print(benchmark.name)
        for config in configs:
            benchmark: Benchmark
            # Note: for 2 DOF and workspace = configuration space
            config["lowLimits"] = [limit[0] for limit in benchmark.collisionChecker.getEnvironmentLimits()]
            config["highLimits"] = [limit[1] for limit in benchmark.collisionChecker.getEnvironmentLimits()]


            fig_local = plt.figure(figsize=(10,10))
            ax = fig_local.add_subplot(1,1,1)
            astar = AStar(benchmark.collisionChecker)

            stats = {}
            title = benchmark.name
            
            start = time.time()
            solution, deltas = astar.planPath(benchmark.startList, benchmark.goalList, config, store_viz=True)
            stats["execution_time"] = time.time() - start

            stats["road_map_size"] = len(astar.graph.nodes.keys())

            stats["num_nodes_solution_path"] = -1

            stats["solution_path_length"] = -1
            
            if solution is not None:
                stats["num_nodes_solution_path"] = len(solution)
                
                stats["solution_path_length"] = astar.graph.nodes[solution[-1]]["g"]

                title += " (No path found!)"

            # save stats
            dir_name = f"{script_dir}/results/{benchmark.name}"
            dir_name = f"{dir_name}/disc{config["discretization"]}_{config["heuristic"]}_w{config["w"]}"
            if config["reopen"]:
                dir_name += "_reopen"
            if config["check_connection"]:
                dir_name += "_linetest"

            if not os.path.exists(dir_name):
                os.makedirs(dir_name)

            stats_filename = f"{dir_name}/stats.json"
            with open(stats_filename, "w") as f:
                json.dump(stats, f)

            deltas_filename = f"{dir_name}/deltas.json"
            with open(deltas_filename, "w") as f:
                json.dump(deltas, f)
            
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
    astarConfig["dof"] = 2
    astarConfig["discretization"] = [50 for _ in range(astarConfig["dof"])]
    astarConfig["check_connection"] = True

    configs = []
    configs.append(astarConfig)
    
    evaluate(configs=configs)

        