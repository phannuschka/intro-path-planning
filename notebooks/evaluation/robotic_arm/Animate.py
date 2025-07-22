import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

import IPTestSuite_robotic_arm_3_DoF as ts3
import IPTestSuite_robotic_arm_6_DoF as ts6
import IPTestSuite_robotic_arm_9_DoF as ts9

from pathlib import Path
import json

from IPVISAStar import aStarVisualizeIncrementalOpenCV
from IPEnvironment import CollisionChecker
from IPBenchmark import Benchmark
from IPAStar import AStar
import networkx as nx

import matplotlib.animation
from IPEnvironmentKin import KinChainCollisionChecker
import numpy as np
import copy
import matplotlib.pyplot as plt
import click

def interpolate_line(startPos, endPos, step_l):
    steps = []
    line = np.array(endPos) - np.array(startPos)
    line_l = np.linalg.norm(line)
    step = line / line_l * step_l
    n_steps = np.floor(line_l / step_l).astype(np.int32)
    c_step = np.array(startPos)
    for i in range(n_steps):
        steps.append(copy.deepcopy(c_step))
        c_step += step
    if not (c_step == np.array(endPos)).all():
        steps.append(np.array(endPos))
    return steps

def planarRobotVisualize(kin_chain, ax):
    joint_positions = kin_chain.get_transforms()
    for i in range(1, len(joint_positions)):
        xs = [joint_positions[i-1][0], joint_positions[i][0]]
        ys = [joint_positions[i-1][1], joint_positions[i][1]]
        ax.plot(xs, ys, color='g')

matplotlib.rcParams['animation.embed_limit'] = 64
def animateSolution(planner, environment: KinChainCollisionChecker, solution, filename, interpolate):
    _planner = planner
    _environment = environment
    _solution = solution
    
    fig_local = plt.figure(figsize=(7, 7))
    ax = fig_local.add_subplot(1, 1, 1)
    ## get positions for solution
    solution_pos = [_planner.graph.nodes[node]['pos'] for node in _solution]
    ## interpolate to obtain a smoother movement
    if interpolate:
        i_solution_pos = [solution_pos[0]]
        for i in range(1, len(solution_pos)):
            segment_s = solution_pos[i-1]
            segment_e = solution_pos[i]
            i_solution_pos = i_solution_pos + interpolate_line(segment_s, segment_e, 0.1)[1:]
    else:
        i_solution_pos = solution_pos
    ## animate
    frames = len(i_solution_pos)
    
    def animate(t):
        ## clear taks space figure
        ax.cla()
        ## fix figure size
        ax.set_xlim([-3,3])
        ax.set_ylim([-3,3])
        ## draw obstacles
        _environment.drawObstacles(ax, inWorkspace = True)
        ## update robot position
        pos = i_solution_pos[t]
        _environment.kin_chain.move(pos)
        planarRobotVisualize(_environment.kin_chain, ax)
    
    ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
    ani.save(filename=filename, writer="ffmpeg", fps=5, bitrate=1800)

@click.command(context_settings=dict(help_option_names=["-h", "--help"]))
@click.option("--path", "-p", type=str, help="The path of the saved results")
@click.option("--interpolate", "-i", default=True, type=bool,  help="Do you want to interpolate between the solution states?")
def animate(path: str, interpolate: bool):

    dir_name = path
    bench_name = Path(dir_name).parent.name
    DoF = Path(dir_name).parent.parent.name
    astar = None

    match DoF:
        case "3DoF":
            bench_list = ts3.benchList

        case "6DoF":
            bench_list = ts6.benchList

        case "9DoF":
            bench_list = ts9.benchList

        case _:
            return


    for b in bench_list:
        if b.name == bench_name:
            astar = AStar(b.collisionChecker)

    if astar is None:
        return
        
    with open(f'{dir_name}/solution.json') as f:
        solution = json.load(f)

    with open(f'{dir_name}/graph.json') as f:
        graph = nx.node_link_graph(json.load(f))
        astar.graph = graph

    output_dir_name = dir_name + "/animation"
    if not os.path.exists(output_dir_name):
        os.makedirs(output_dir_name)

    output_name = output_dir_name + "/animation.mp4"

    animateSolution(planner=astar, environment=astar._collisionChecker, solution=solution, filename=output_name, interpolate=interpolate)



if __name__ == "__main__":
    animate()

