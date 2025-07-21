import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
notebooks_dir = os.path.join(script_dir, "..", "..")
sys.path.append(os.path.abspath(notebooks_dir))

from pathlib import Path
import json

from IPVISAStar import aStarVisualizeIncrementalOpenCV
from IPTestSuite_2DoF import benchList
from IPEnvironment import CollisionChecker
from IPBenchmark import Benchmark
from IPAStar import AStar
import networkx as nx

import click

@click.command(context_settings=dict(help_option_names=["-h", "--help"]))
@click.option("--path", "-p", type=str, help="The path of the saved results")
def animate(path: str):

    dir_name = path
    bench_name = Path(dir_name).parent.name
    print(bench_name)
    astar = None

    for b in benchList:
        if b.name == bench_name:
            astar = AStar(b.collisionChecker)

    if astar is None:
        return
        
    with open(f'{dir_name}/solution.json') as f:
        solution = json.load(f)

    with open(f'{dir_name}/deltas.json') as f:
        deltas = json.load(f)

    with open(f'{dir_name}/graph.json') as f:
        graph = nx.node_link_graph(json.load(f))
        astar.graph = graph

    output_dir_name = dir_name + "/animation"

    aStarVisualizeIncrementalOpenCV(astar, solution, deltas, output_dir_name)



if __name__ == "__main__":
    animate()
