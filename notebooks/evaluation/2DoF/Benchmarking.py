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

astarConfig = dict()
astarConfig["w"] = .5
astarConfig["heuristic"]  = "euclidean"
astarConfig["reopen"] = True
astarConfig["dof"] = 2
astarConfig["discretization"] = [50 for _ in range(astarConfig["dof"])]
astarConfig["check_connection"] = True

configs = []
configs.append(astarConfig)

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
        solution, deltas = astar.planPath(benchmark.startList, benchmark.goalList, astarConfig)
        stats["execution_time"] = time.time() - start

        stats["road_map_size"] = len(astar.graph.nodes.keys())

        stats["num_nodes_solution_path"] = -1

        stats["solution_path_length"] = -1
        
        if solution is not None:
            stats["num_nodes_solution_path"] = len(solution)
            
            stats["solution_path_length"] = astar.graph.nodes[solution[-1]]["g"]

            title += " (No path found!)"

        # save stats
        dirname = f"{script_dir}/results/{benchmark.name}"
        if not os.path.exists(dirname):
            os.makedirs(dirname)
        filename = f"{dirname}/disc{astarConfig["discretization"]}_{astarConfig["heuristic"]}_w{astarConfig["w"]}"
        if astarConfig["reopen"]:
            filename += "_reopen"
        if astarConfig["check_connection"]:
            filename += "_linetest"
        filename += ".json"
        with open(filename, "w") as f:
            json.dump(stats, f)

        