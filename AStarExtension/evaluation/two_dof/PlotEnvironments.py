import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from IPBenchmark import Benchmark 
import evaluation.two_dof.IPTestSuite_2DoF as ts

import matplotlib.pyplot as plt


def visualizeBenchmark(benchmark: Benchmark, discretizationX: int | None = None, discretizationY: int | None = None):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(1, 1, 1)
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

    if discretizationX is not None and discretizationY is not None:
        axis_length_x = limits[0][1] - limits[0][0]
        discretization_step_x = axis_length_x / discretizationX
        axis_length_y = limits[1][1] - limits[1][0]
        discretization_step_y = axis_length_y / discretizationY

        # Calculate how many steps to go in each direction from start point
        start_x, start_y = benchmark.startList[0][0], benchmark.startList[0][1]

        # Calculate range of grid indices to cover the plot area
        steps_left = int((start_x - limits[0][0]) / discretization_step_x) + 1
        steps_right = int((limits[0][1] - start_x) / discretization_step_x) + 1
        steps_down = int((start_y - limits[1][0]) / discretization_step_y) + 1
        steps_up = int((limits[1][1] - start_y) / discretization_step_y) + 1

        # Create grid points
        x_grid = []
        y_grid = []

        for i in range(-steps_left, steps_right + 1):
            x_pos = start_x + i * discretization_step_x
            if limits[0][0] <= x_pos <= limits[0][1]:  # Only include points within limits
                for j in range(-steps_down, steps_up + 1):
                    y_pos = start_y + j * discretization_step_y
                    if limits[1][0] <= y_pos <= limits[1][1]:  # Only include points within limits
                        x_grid.append(x_pos)
                        y_grid.append(y_pos)

        # Plot grid points
        ax.scatter(x_grid, y_grid, s=10, c='gray', alpha=0.5, zorder=1)



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

    return fig, ax

def main():
    dir_name = f"{os.path.dirname(os.path.abspath(__file__))}/environment_viz"
    if not os.path.exists(dir_name):
        os.makedirs(dir_name)

    for benchmark in ts.benchList:
        benchmark: Benchmark
 
        fig, ax = visualizeBenchmark(benchmark)

        file_name = f"{dir_name}/{benchmark.name}.svg"
        fig.savefig(file_name)


if __name__ == "__main__":
    main()