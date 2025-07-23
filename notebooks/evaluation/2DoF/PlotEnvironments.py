import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from IPBenchmark import Benchmark 
import IPTestSuite_2DoF as ts

import matplotlib.pyplot as plt
import networkx as nx 


def main():
    dir_name = f"{os.path.dirname(os.path.abspath(__file__))}/environment_viz"
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)

    for benchmark in ts.benchList:
        benchmark: Benchmark
 
        fig = plt.figure(figsize=(5,5))
        ax = fig.add_subplot(1,1,1)
        limits = benchmark.collisionChecker.getEnvironmentLimits()
        ax.set_xlim(limits[0])
        ax.set_ylim(limits[1])
        benchmark.collisionChecker.drawObstacles(ax)

        ax.scatter(
            benchmark.startList[0][0],
            benchmark.startList[0][1],
            s=200,
            c='#00dd00',
        )

        ax.text(
            benchmark.startList[0][0],
            benchmark.startList[0][1],
            "S",
            size=12,
            color="k",
            family="sans-serif",
            weight="normal",
            horizontalalignment="center",
            verticalalignment="center")
        
        ax.scatter(
            benchmark.goalList[0][0],
            benchmark.goalList[0][1],
            s=200,
            c='#dd0000',
        )

        ax.text(
            benchmark.goalList[0][0],
            benchmark.goalList[0][1],
            "G",
            size=12,
            color="k",
            family="sans-serif",
            weight="normal",
            horizontalalignment="center",
            verticalalignment="center")

        file_name = f"{dir_name}/{benchmark.name}.png"
        fig.savefig(file_name)


if __name__ == "__main__":
    main()