import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from IPBenchmark import Benchmark 
import IPTestSuite_robotic_arm_3_DoF as ts

import matplotlib.pyplot as plt


def planarRobotVisualize(kin_chain, ax, color):
    joint_positions = kin_chain.get_transforms()
    for i in range(1, len(joint_positions)):
        xs = [joint_positions[i-1][0], joint_positions[i][0]]
        ys = [joint_positions[i-1][1], joint_positions[i][1]]
        ax.plot(xs, ys, color=color)


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
        benchmark.collisionChecker.drawObstacles(ax, inWorkspace=True)

        r = benchmark.collisionChecker.kin_chain
        r.move(benchmark.startList[0])
        planarRobotVisualize(r, ax, "g")
        r.move(benchmark.goalList[0])
        planarRobotVisualize(r, ax, "r")

        file_name = f"{dir_name}/{benchmark.name}.svg"
        fig.savefig(file_name)


if __name__ == "__main__":
    main()