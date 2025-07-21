# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

import networkx as nx
import matplotlib.pyplot as plt
from IPython.core.display_functions import display


def aStarVisualize(planner, solution, ax = None, nodeSize = 300, viz_solution=True, viz_obstacles=True):
    graph = planner.graph
    collChecker = planner._collisionChecker
    # get a list of positions of all nodes by returning the content of the attribute 'pos'
    pos = nx.get_node_attributes(graph,'pos')
    color = nx.get_node_attributes(graph,'color')
    
    # get a list of degrees of all nodes
    #degree = nx.degree_centrality(graph)
    
    # draw graph (nodes colorized by degree)
    open_nodes = [node for node,attribute in graph.nodes(data=True) if attribute['status']=="open" and (attribute['collision']==0)]
    draw_nodes = nx.draw_networkx_nodes(graph, pos, node_color='#FFFFFF', nodelist=open_nodes, ax = ax, node_size=nodeSize)
    draw_nodes.set_edgecolor("b")
    closed_nodes = [node for node,attribute in graph.nodes(data=True) if attribute['status']=="closed" and attribute['collision']==0]
    draw_nodes = nx.draw_networkx_nodes(graph, pos, node_color='#0000FF', nodelist=closed_nodes, ax = ax, node_size=nodeSize)
    colliding_nodes = [node for node,attribute in graph.nodes(data=True) if attribute['collision']==1]
    draw_nodes = nx.draw_networkx_nodes(graph, pos, node_color='#FF0000', nodelist=colliding_nodes, ax = ax, node_size=nodeSize)
    #nx.draw_networkx_nodes(graph, pos,  cmap=plt.cm.Blues, ax = ax, node_size=nodeSize)
    nx.draw_networkx_edges(graph,pos,
                               edge_color='b',
                               width=3.0
                            )

    if viz_obstacles:
        collChecker.drawObstacles(ax)

    if viz_solution:
        # draw nodes based on solution path
        Gsp = nx.subgraph(graph,solution)
        nx.draw_networkx_nodes(Gsp,pos,
                                node_size=nodeSize,
                                 node_color='g')

        # draw edges based on solution path
        nx.draw_networkx_edges(Gsp,pos,alpha=0.8,edge_color='g',width=10,arrows=True)

        nx.draw_networkx_nodes(graph,pos,nodelist=[solution[0]],
                               node_size=300,
                               node_color='#00dd00',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={solution[0]: "S"},  ax = ax)


        nx.draw_networkx_nodes(graph,pos,nodelist=[solution[-1]],
                                       node_size=300,
                                       node_color='#DD0000',  ax = ax)
        nx.draw_networkx_labels(graph,pos,labels={solution[-1]: "G"},  ax = ax)


def aStarVisualizeSteps(planner, solution, graphs, ax = None, nodeSize = 300, step=0):
    print("Visualizing step", step, "of", len(graphs)-1)
    graph = graphs[step]
    planner.graph = graph

    aStarVisualize(planner, solution, ax = ax, nodeSize = nodeSize, viz_solution=step==len(graphs)-1, viz_obstacles=True)

    #  draw start and goal
    final_graph = graphs[-1]
    pos = nx.get_node_attributes(final_graph, 'pos')
    nx.draw_networkx_nodes(final_graph, pos, nodelist=[solution[0]],
                           node_size=300,
                           node_color='#00dd00', ax=ax)
    nx.draw_networkx_labels(final_graph, pos, labels={solution[0]: "S"}, ax=ax)

    nx.draw_networkx_nodes(final_graph, pos, nodelist=[solution[-1]],
                           node_size=300,
                           node_color='#DD0000', ax=ax)
    nx.draw_networkx_labels(final_graph, pos, labels={solution[-1]: "G"}, ax=ax)


import matplotlib.animation
from IPython.display import HTML
import os

matplotlib.rcParams['animation.embed_limit'] = 2048


def animateSolution(astar, solution, graphs, filename="astar_animation.mp4", notebook: bool = True):
    fig_local = plt.figure(figsize=(5, 5))
    ax = fig_local.add_subplot(1, 1, 1)
    ax.set_xlim(astar._collisionChecker.getEnvironmentLimits()[0])
    ax.set_ylim(astar._collisionChecker.getEnvironmentLimits()[1])

    print("Visualizing A* solution with", len(graphs), "steps..")

    ## animate
    frames = len(graphs)

    def animate(step):
        ## clear figure
        ax.cla()

        ## draw plot
        aStarVisualizeSteps(astar, solution, ax=ax, nodeSize=10, step=step, graphs=graphs)
        ax.set_title(f"Step {step}")

    ani = matplotlib.animation.FuncAnimation(fig_local, animate, frames=frames)
    # ani.save(filename, writer='ffmpeg', fps=5, bitrate=1800)
    if notebook:
        html = HTML(ani.to_jshtml())
        display(html)
        plt.close()
    else:
        plt.show()

def save_step_images(planner, solution, graphs, output_dir="steps"):
    os.makedirs(output_dir, exist_ok=True)
    for i, graph in enumerate(graphs):
        fig, ax = plt.subplots(figsize=(10, 10))
        aStarVisualizeSteps(planner, solution, graphs, ax=ax, step=i, nodeSize=10)
        plt.savefig(f"{output_dir}/step_{i:03d}.png", dpi=150, bbox_inches='tight')
        plt.close()





