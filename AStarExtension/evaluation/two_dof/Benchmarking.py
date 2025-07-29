import copy
import sys
import os

from scipy.cluster.hierarchy import weighted

script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..", "core")
sys.path.append(os.path.abspath(notebooks_dir))

import IPTestSuite_2DoF as ts
from IPEnvironment import CollisionChecker
from IPBenchmark import Benchmark
import matplotlib.pylab as plt
from shapely.geometry import Point, Polygon, LineString
from shapely import plotting
from IPAStarExtended import AStar
from IPVISAStar import aStarVisualize
from IPPerfMonitor import IPPerfMonitor
import time
import json
import networkx as nx
from typing import List, Dict

def get_config_dir_name(config: Dict, benchmark_name: str):
    dir_name = f"{script_dir}/results/{benchmark_name}"
    dir_name = f"{dir_name}/disc{config['discretization']}_{config['heuristic']}_w{config['w']}"
    if config["reopen"]:
        dir_name += "_reopen"
    if config["check_connection"]:
        dir_name += "_linetest"
        if config["lazy_check_connection"]:
            dir_name += "_lazy"
    
    return dir_name

def evaluate(config: Dict, benchmark: Benchmark, dump: bool = True):
    benchmark: Benchmark
    # Note: for 2 DOF and workspace = configuration space
    config["lowLimits"] = [limit[0] for limit in benchmark.collisionChecker.getEnvironmentLimits()]
    config["highLimits"] = [limit[1] for limit in benchmark.collisionChecker.getEnvironmentLimits()]

    astar = AStar(benchmark.collisionChecker)

    stats = {}
    
    start = time.time()
    solution, deltas = astar.planPath(benchmark.startList, benchmark.goalList, config, store_viz=True)
    stats["execution_time"] = time.time() - start

    stats["road_map_size"] = len(astar.graph.nodes.keys())

    stats["num_nodes_solution_path"] = -1

    stats["solution_path_length"] = -1
    
    if solution != []:
        stats["num_nodes_solution_path"] = len(solution)
        
        stats["solution_path_length"] = astar.graph.nodes[solution[-1]]["g"]

    # save stats
    if dump:
        dir_name = get_config_dir_name(config=config, benchmark_name=benchmark.name)

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
            json.dump(nx.node_link_data(astar.graph, edges="links"), f)

        solution_filename = f"{dir_name}/solution.json"
        with open(solution_filename, "w") as f:
            json.dump(solution, f)

    return stats, solution, deltas, astar 


def evaluate_all(configs: List[Dict]):
    for config in configs:
        benchmarks = [ts.benchList[i] for i in config["benchmarks"]]
        for benchmark in benchmarks:
            print(benchmark.name)
            benchmark: Benchmark
            _ = evaluate(config=config, benchmark=benchmark, dump=True)


if __name__ == "__main__":
    configs = []

    # Note: config for testing
    # astarConfig = dict()
    # astarConfig["w"] = .5
    # astarConfig["heuristic"]  = "euclidean"
    # astarConfig["reopen"] = True
    # astarConfig["dof"] = 2
    # astarConfig["discretization"] = [50 for _ in range(astarConfig["dof"])]
    # astarConfig["check_connection"] = True
    # astarConfig["lazy_check_connection"] = True
    # astarConfig["benchmarks"] = [0, 1, 2, 3, 4, 5]
    # configs.append(astarConfig)

    #-----------------------------------------------------------------------------------------------

    discretization_config = dict()
    discretization_config["w"] = .5
    discretization_config["heuristic"]  = "euclidean"
    discretization_config["reopen"] = True
    discretization_config["dof"] = 2
    discretization_config["check_connection"] = True
    discretization_config["lazy_check_connection"] = True
    discretization_config["benchmarks"] = [0, 1, 2, 3, 6]

    for discretization_value in [10, 20, 50, 100, 200]:
        config = copy.deepcopy(discretization_config)
        config["discretization"] = [discretization_value for _ in range(config["dof"])]
        configs.append(config)

    #-----------------------------------------------------------------------------------------------

    weight_config = dict()
    weight_config["heuristic"] = "euclidean"
    weight_config["reopen"] = True
    weight_config["dof"] = 2
    weight_config["discretization"] = [50 for _ in range(weight_config["dof"])]
    weight_config["check_connection"] = True
    weight_config["lazy_check_connection"] = True
    weight_config["benchmarks"] = [0, 1, 2, 3, 6]
    
    for w_value in [0.5, 0.75, 1.0]:
        config = copy.deepcopy(weight_config)
        config["w"] = w_value
        configs.append(config)

    #-----------------------------------------------------------------------------------------------

    reopen_config = dict()
    reopen_config["w"] = 0.75
    reopen_config["heuristic"] = "euclidean"
    reopen_config["dof"] = 2
    reopen_config["discretization"] = [100 for _ in range(reopen_config["dof"])]
    reopen_config["check_connection"] = True
    reopen_config["lazy_check_connection"] = True
    reopen_config["benchmarks"] = [0, 1, 2, 3, 6]
    for reopen_value in [True, False]:
        config = copy.deepcopy(reopen_config)
        config["reopen"] = reopen_value
        configs.append(config)

    #-----------------------------------------------------------------------------------------------
    # Note: test for presentation
    # astarConfig = dict()
    # astarConfig["w"] = .5
    # astarConfig["heuristic"]  = "euclidean"
    # astarConfig["reopen"] = False
    # astarConfig["dof"] = 2
    # astarConfig["discretization"] = [2.1 for _ in range(astarConfig["dof"])]
    # astarConfig["check_connection"] = False
    # astarConfig["lazy_check_connection"] = True
    # astarConfig["benchmarks"] = [8]
    # configs.append(astarConfig)

    #-----------------------------------------------------------------------------------------------

    print(f"Evaluating {len(configs)} configs...")
    evaluate_all(configs=configs)

        