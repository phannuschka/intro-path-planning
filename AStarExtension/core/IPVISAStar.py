# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""
import gc
import time

import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
from IPython.core.display_functions import display


def aStarVisualize(planner, solution, ax = None, nodeSize = 300, viz_solution=True, viz_obstacles=True):
    start = time.time()
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
    print(f"Drawing graph took {time.time() - start:.4f} seconds")

    if viz_obstacles:
        start = time.time()
        collChecker.drawObstacles(ax)
        print(f"Drawing obstacles took {time.time() - start:.4f} seconds")

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

    start = time.time()
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
    print(f"Drawing start and goal took {time.time() - start:.4f} seconds")


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


def save_step_images(planner, solution, graphs, output_dir="steps"):
    os.makedirs(output_dir, exist_ok=True)
    for i, graph in enumerate(graphs):
        fig, ax = plt.subplots(figsize=(10, 10))
        aStarVisualizeSteps(planner, solution, graphs, ax=ax, step=i, nodeSize=10)
        plt.savefig(f"{output_dir}/step_{i:03d}.png", dpi=150, bbox_inches='tight')
        plt.close()


def aStarVisualizeIncremental(planner, solution, deltas, ax=None, nodeSize=10, output_dir="steps"):
    """
    Visualize the A* algorithm in an incremental way.

    Args:
        planner: The A* planner instance.
        solution: The solution path.
        deltas: List of delta operations at each step.
        ax: Matplotlib axis to draw on.
        nodeSize: Size of the nodes in the graph.
    """

    os.makedirs(output_dir, exist_ok=True)
    matplotlib.use('Agg')

    graph = planner.graph
    collChecker = planner._collisionChecker

    # Create figure once and reuse
    fig, ax = plt.subplots(figsize=(10, 10))

    # Pre-compute position dictionary once
    pos = nx.get_node_attributes(graph, 'pos')

    # Draw static elements once (obstacles, start, goal)
    collChecker.drawObstacles(ax)

    # Draw start and goal
    nx.draw_networkx_nodes(graph, pos, nodelist=[solution[0]],
                           node_size=300, node_color='#00dd00', ax=ax)
    nx.draw_networkx_labels(graph, pos, labels={solution[0]: "S"}, ax=ax)

    nx.draw_networkx_nodes(graph, pos, nodelist=[solution[-1]],
                           node_size=300, node_color='#DD0000', ax=ax)
    nx.draw_networkx_labels(graph, pos, labels={solution[-1]: "G"}, ax=ax)

    # Track drawn elements to avoid redrawing
    drawn_nodes = set([solution[0], solution[-1]])
    drawn_edges = set()

    # Group deltas by iteration (fixed version)
    i = 0
    while i < len(deltas):
        delta = deltas[i]
        iteration = delta['iteration']
        print(f"Processing iteration {iteration}")

        # Get all deltas for the current iteration
        deltas_iteration = [delta]
        j = i + 1
        while j < len(deltas) and deltas[j]['iteration'] == iteration:
            deltas_iteration.append(deltas[j])
            j += 1
        i = j  # Move to next different iteration

        # Batch process all deltas for this iteration
        nodes_to_draw = {}  # {node_name: (color, size)}
        edges_to_draw = []
        edges_to_remove = []

        for delta in deltas_iteration:
            action_type = delta['action_type']

            if action_type in ['add_node', 'update_node']:
                node_name = delta['node_name']
                if node_name in graph.nodes:
                    node = graph.nodes[node_name]

                    # Determine color and size
                    if node.get('collision', 0) == 1:
                        color = '#FF4444'
                    elif node.get('line_collision', 0) == 1:
                        color = '#FF8800'
                    elif node.get('status', '') == 'closed':
                        color = '#0000FF'
                    elif node.get('status', '') == 'open':
                        color = '#FFFFFF'
                    else:
                        color = '#000000'

                    nodes_to_draw[node_name] = (color, nodeSize)

            elif action_type == 'close_node':
                node_name = delta['node_name']
                if node_name in graph.nodes and node_name not in [solution[0], solution[-1]]:
                    node = graph.nodes[node_name]

                    # Determine color and size for closed nodes
                    if delta.get('current_best', False):
                        if node.get('collision', 0) == 1:
                            color = '#FF0000'
                        elif node.get('line_collision', 0) == 1:
                            color = '#FF6600'
                        else:
                            color = '#000000'
                        size = nodeSize * 2
                    else:
                        color = '#0000FF'
                        size = nodeSize * 2

                    nodes_to_draw[node_name] = (color, size)

            elif action_type == 'add_edge':
                from_node = delta['from_node']
                to_node = delta['to_node']
                if from_node in graph.nodes and to_node in graph.nodes:
                    edge = (to_node, from_node)
                    if edge not in drawn_edges:
                        edges_to_draw.append(edge)
                        drawn_edges.add(edge)

            elif action_type == 'remove_edge':
                from_node = delta['from_node']
                to_node = delta['to_node']
                if from_node in graph.nodes and to_node in graph.nodes:
                    edge = (to_node, from_node)
                    edges_to_remove.append(edge)

        # Batch draw nodes by color and size to minimize draw calls
        nodes_by_style = {}  # {(color, size): [node_list]}
        for node_name, (color, size) in nodes_to_draw.items():
            if node_name not in drawn_nodes:
                drawn_nodes.add(node_name)
                key = (color, size)
                if key not in nodes_by_style:
                    nodes_by_style[key] = []
                nodes_by_style[key].append(node_name)

        # Draw nodes in batches
        for (color, size), node_list in nodes_by_style.items():
            if node_list:
                node_positions = {node: pos[node] for node in node_list}
                nx.draw_networkx_nodes(graph, node_positions, nodelist=node_list,
                                       node_size=size, node_color=color, ax=ax)

        # Draw edges in batch
        if edges_to_draw:
            nx.draw_networkx_edges(graph, pos, edgelist=edges_to_draw,
                                   edge_color='#CCCCCC', width=1, ax=ax)

        # Draw removed edges in batch
        if edges_to_remove:
            nx.draw_networkx_edges(graph, pos, edgelist=edges_to_remove,
                                   edge_color='#FF0000', width=1, ax=ax)

        # Update iteration text (remove previous text first)
        ax.texts.clear()  # Remove previous iteration text
        ax.text(0.02, 0.98, f'Iteration: {iteration}', transform=ax.transAxes,
                fontsize=12, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        # Save with optimized settings
        fig.savefig(f"{output_dir}/step_{iteration:03d}.jpg",
                    dpi=100, bbox_inches=None, quality=85)

        # Periodic cleanup to prevent memory buildup
        if iteration % 50 == 0:
            import gc
            gc.collect()

    plt.close(fig)


def aStarVisualizeIncremental2(planner, solution, deltas, nodeSize=10, output_dir="steps"):
    """
    Visualize the A* algorithm in an incremental way.

    Args:
        planner: The A* planner instance.
        solution: The solution path.
        graphs: List of graphs at each step.
        ax: Matplotlib axis to draw on.
        nodeSize: Size of the nodes in the graph.
    """

    os.makedirs(output_dir, exist_ok=True)

    matplotlib.use('Agg')
    graph = planner.graph
    collChecker = planner._collisionChecker

    fig, ax = plt.subplots(figsize=(10, 10))
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)

    ax.set_xlim(collChecker.getEnvironmentLimits()[0])
    ax.set_ylim(collChecker.getEnvironmentLimits()[1])
    ax.axis("off")
    # draw obstacles
    collChecker.drawObstacles(ax)

    # draw start and goal
    pos = nx.get_node_attributes(graph, 'pos')
    nx.draw_networkx_nodes(graph, pos, nodelist=[solution[0]],
                           node_size=300,
                           node_color='#00dd00', ax=ax)
    nx.draw_networkx_labels(graph, pos, labels={solution[0]: "S"}, ax=ax)

    nx.draw_networkx_nodes(graph, pos, nodelist=[solution[-1]],
                           node_size=300,
                           node_color='#DD0000', ax=ax)
    nx.draw_networkx_labels(graph, pos, labels={solution[-1]: "G"}, ax=ax)

    initial_path = f"{output_dir}/step_000.png"
    # plt.savefig(initial_path, dpi=150, pad_inches=0, bbox_inches='tight')
    fig.canvas.draw()
    graphic = np.array(fig.canvas.renderer.buffer_rgba())
    plt.imsave(initial_path, graphic)
    previous_image_path = initial_path

    plt.close()

    i = 0
    while i < len(deltas):
        start = time.time()
        delta = deltas[i]
        iteration = delta['iteration']
        print(f"Processing iteration {iteration}")

        # get all deltas for the current iteration
        deltas_iteration = [delta]
        for j in range(i + 1, len(deltas)):
            if deltas[j]['iteration'] == iteration:
                deltas_iteration.append(deltas[j])
            else:
                i = j
                break
        print(f"Getting {iteration} took {time.time() - start:.4f} seconds")

        background = plt.imread(previous_image_path)
        fig, ax = plt.subplots(figsize=(10, 10))
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        ax.set_xlim(collChecker.getEnvironmentLimits()[0])
        ax.set_ylim(collChecker.getEnvironmentLimits()[1])
        ax.axis('off')
        xlim = collChecker.getEnvironmentLimits()[0]
        ylim = collChecker.getEnvironmentLimits()[1]
        ax.imshow(background, extent=[xlim[0], xlim[1], ylim[0], ylim[1]], aspect='auto', alpha=1.0)

        for delta in deltas_iteration:
            action_type = delta['action_type']
            if action_type == 'add_node':
                node_name = delta['node_name']
                start = time.time()
                if node_name in graph.nodes:
                    node = graph.nodes[node_name]
                    pos_dict = {node_name: node['pos']}

                    if node.get('collision', 0) == 1:
                        color = '#FF4444'
                    elif node.get('line_collision', 0) == 1:
                        color = '#FF8800'
                    elif node.get('status', '') == 'closed':
                        color = '#0000FF'
                    elif node.get('status', '') == 'open':
                        color = '#FFFFFF'
                    else:
                        color = '#000000'
                    nx.draw_networkx_nodes(graph, pos_dict, nodelist=[node_name], node_size=nodeSize, node_color=color, ax=ax)
                print(f"Drawing node took {time.time() - start:.4f} seconds")

            elif action_type == 'update_node':
                node_name = delta['node_name']
                start = time.time()
                if node_name in graph.nodes:
                    node = graph.nodes[node_name]
                    pos_dict = {node_name: node['pos']}

                    if node.get('collision', 0) == 1:
                        color = '#FF4444'
                    elif node.get('line_collision', 0) == 1:
                        color = '#FF8800'
                    elif node.get('status', '') == 'closed':
                        color = '#0000FF'
                    elif node.get('status', '') == 'open':
                        color = '#FFFFFF'
                    else:
                        color = '#000000'
                    nx.draw_networkx_nodes(graph, pos_dict, nodelist=[node_name], node_size=nodeSize, node_color=color, ax=ax)
                print(f"Updating node took {time.time() - start:.4f} seconds")


            elif action_type == 'close_node':
                start = time.time()
                node_name = delta['node_name']
                if node_name in graph.nodes and node_name not in [solution[0], solution[-1]]:
                    node = graph.nodes[node_name]
                    pos_dict = {node_name: node['pos']}

                    # Highlight current best node with larger size and special color
                    if delta.get('current_best', False):
                        if node.get('collision', 0) == 1:
                            color = '#FF0000'
                            size = nodeSize * 2
                        elif node.get('line_collision', 0) == 1:
                            color = '#FF6600'
                            size = nodeSize * 2
                        else:
                            color = '#000000'
                            size = nodeSize * 2
                    else:
                        color = '#0000FF'
                        size = nodeSize * 2
                    nx.draw_networkx_nodes(graph, pos_dict, nodelist=[node_name],
                                           node_size=size, node_color=color, ax=ax)
                print(f"Closing node took {time.time() - start:.4f} seconds")

            elif action_type == 'add_edge':
                from_node = delta['from_node']
                to_node = delta['to_node']
                if from_node in graph.nodes and to_node in graph.nodes:
                    edge_pos = {from_node: pos[from_node], to_node: pos[to_node]}
                    start = time.time()
                    nx.draw_networkx_edges(graph, edge_pos, edgelist=[(to_node, from_node)],
                                           edge_color='#CCCCCC', width=1, ax=ax)
                    print(f"Adding edge took {time.time() - start:.4f} seconds")

            elif action_type == 'remove_edge':
                start = time.time()
                from_node = delta['from_node']
                to_node = delta['to_node']
                if from_node in graph.nodes and to_node in graph.nodes:
                    edge_pos = {from_node: pos[from_node], to_node: pos[to_node]}
                    nx.draw_networkx_edges(graph, edge_pos, edgelist=[(to_node, from_node)],
                                           edge_color='#FF0000', width=1, ax=ax)
                print(f"Removing edge took {time.time() - start:.4f} seconds")

            else:
                print(f"Unknown action type: {action_type}")

        # draw iteration
        ax.text(0.02, 0.98, f'Iteration: {iteration}', transform=ax.transAxes,
                fontsize=12, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        start = time.time()
        current_path = f"{output_dir}/step_{iteration:03d}.png"
        # plt.savefig(current_path, dpi=150, pad_inches=0, bbox_inches='tight')
        fig.canvas.draw()
        graphic = np.array(fig.canvas.renderer.buffer_rgba())
        plt.imsave(initial_path, graphic)
        print(f"Saving step image took {time.time() - start:.4f} seconds")

        plt.close()
        previous_image_path = current_path

        if iteration % 50 == 0:
            gc.collect()


    plt.close()


import cv2
import numpy as np
import os

def aStarVisualizeIncrementalOpenCV(planner, solution, deltas, output_dir="steps_fast"):
    """
    Fast, persistent OpenCV-based visualization of A* search with proper scaling.
    Only updates what's new, and uses planner environment limits for scaling.

    Args:
        planner: A* planner instance.
        solution: List of nodes representing the solution path.
        deltas: List of delta updates (with iteration & action_type).
        output_dir: Folder to save frames.
    """
    os.makedirs(output_dir, exist_ok=True)
    graph = planner.graph
    pos = nx.get_node_attributes(graph, 'pos')
    limits = planner._collisionChecker.getEnvironmentLimits()
    xmin, xmax, ymin, ymax = limits[0][0], limits[0][1], limits[1][0], limits[1][1]

    # Margin and scaling
    margin = 10
    img_width = 2000
    scale_x = ((img_width - 2 * margin) / (xmax - xmin))
    img_height = int((ymax - ymin) * scale_x) + 2 * margin
    scale_y = scale_x  # uniform scaling

    # Coordinate transform
    def to_pixel(p):
        x, y = p
        px = int((x - xmin) * scale_x + margin)
        py = int(img_height - ((y - ymin) * scale_y + margin))
        return (px, py)

    def draw_arrow(img, start, end, color=(100, 100, 100), thickness=5):
        # direction = np.array(end).astype(float) - np.array(start).astype(float)
        # direction /= np.linalg.norm(direction)
        # new_start, new_end = [0, 0], [0, 0]
        # new_start[0] = start[0] + int(direction[0] * 5)
        # new_start[1] = start[1] + int(direction[1] * 5)
        # new_end[0] = end[0] - int(direction[0] * 5)
        # new_end[1] = end[1] - int(direction[1] * 5)
        # cv2.arrowedLine(img, start, end, color, thickness, tipLength=0.5)
        direction = np.array(end).astype(float) - np.array(start).astype(float)
        direction /= np.linalg.norm(direction)

        # Find midpoint
        mid_x = (start[0] + end[0]) // 2
        mid_y = (start[1] + end[1]) // 2

        # Triangle size (similar to node radius of 10)
        triangle_size = 10

        # Create perpendicular vector for triangle base
        perp = np.array([-direction[1], direction[0]])

        # Calculate triangle vertices
        # Tip of triangle (pointing in direction)
        tip = (int(mid_x + direction[0] * triangle_size),
               int(mid_y + direction[1] * triangle_size))

        # Base vertices (perpendicular to direction)
        base1 = (int(mid_x - direction[0] * triangle_size / 2 + perp[0] * triangle_size / 2),
                 int(mid_y - direction[1] * triangle_size / 2 + perp[1] * triangle_size / 2))

        base2 = (int(mid_x - direction[0] * triangle_size / 2 - perp[0] * triangle_size / 2),
                 int(mid_y - direction[1] * triangle_size / 2 - perp[1] * triangle_size / 2))

        # Draw filled triangle
        triangle_pts = np.array([tip, base1, base2], np.int32)
        cv2.fillPoly(img, [triangle_pts], color)

    # Create persistent frame with obstacles and static nodes
    frame = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

    # Draw obstacles (if method exists)
    if hasattr(planner._collisionChecker, 'drawOnImage'):
        frame = planner._collisionChecker.drawOnImage(frame.copy(), scale=scale_x, offset=(margin, margin))

    # Draw start and goal nodes
    start = to_pixel(planner.start)
    goal = to_pixel(planner.goal)
    cv2.circle(frame, start, 20, (0, 255, 0), -1)
    cv2.putText(frame, 'S', (start[0] + 5, start[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 100, 0), 3)
    cv2.circle(frame, goal, 20, (0, 0, 255), -1)
    cv2.putText(frame, 'G', (goal[0] + 5, goal[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 100), 3)

    # Track drawn content
    drawn_nodes = set()# [solution[0], solution[-1]])
    drawn_edges = set()

    i = 0
    while i < len(deltas):
        delta = deltas[i]
        iteration = delta['iteration']
        deltas_iteration = [delta]
        j = i + 1
        while j < len(deltas) and deltas[j]['iteration'] == iteration:
            deltas_iteration.append(deltas[j])
            j += 1
        i = j

        for delta in deltas_iteration:
            action = delta['action_type']

            if action == 'add_node':
                node_name = delta['node_name']
                status = delta.get('status', None)
                if node_name in pos and status is not None:
                    if delta.get('collision', 0) == 1:
                        color = (0, 0, 255)
                    elif delta.get('line_collision', 0) == 1:
                        color = (0, 165, 255)
                    elif status == 'closed':
                        color = (255, 0, 0)
                    elif status == 'open':
                        color = (255, 200, 200)
                    else:
                        color = (0, 0, 0)

                    px, py = to_pixel(pos[node_name])
                    cv2.circle(frame, (px, py), 10, color, -1)
                    drawn_nodes.add(node_name)

            elif action == 'update_node':
                node_name = delta['node_name']
                status = delta.get('status', None)
                if node_name in pos and status is not None:
                    if delta.get('collision', 0) == 1:
                        color = (0, 0, 255)
                    elif delta.get('line_collision', 0) == 1:
                        color = (0, 165, 255)
                    elif status == 'closed':
                        color = (255, 0, 0)
                    elif status == 'open':
                        color = (255, 200, 200)
                    else:
                        color = (0, 0, 0)

                    px, py = to_pixel(pos[node_name])
                    cv2.circle(frame, (px, py), 10, color, -1)
                    drawn_nodes.add(node_name)

            elif action == 'close_node':
                node_name = delta['node_name']
                status = delta.get('status', None)
                if node_name in pos and status is not None:
                    if delta.get('collision', 0) == 1:
                        color = (0, 0, 255)
                    elif delta.get('line_collision', 0) == 1:
                        color = (0, 165, 255)
                    elif status == 'closed':
                        color = (255, 0, 0)
                    elif status == 'open':
                        color = (255, 200, 200)
                    else:
                        color = (0, 0, 0)

                    px, py = to_pixel(pos[node_name])
                    cv2.circle(frame, (px, py), 10, color, -1)
                    drawn_nodes.add(node_name)
                else:
                    print(f"Node {node_name} not found in position dictionary or no status available.")
                    print(f"delta: {delta}")

            elif action == 'add_edge':
                n1 = delta['from_node']
                n2 = delta['to_node']
                edge = (n1, n2)
                if n1 in pos and n2 in pos:
                    p1 = to_pixel(pos[n1])
                    p2 = to_pixel(pos[n2])
                    draw_arrow(frame, p1, p2)
                    drawn_edges.add(edge)

            elif action == 'remove_edge':
                n1 = delta['from_node']
                n2 = delta['to_node']
                edge = (n1, n2)
                if n1 in pos and n2 in pos:
                    p1 = to_pixel(pos[n1])
                    p2 = to_pixel(pos[n2])
                    draw_arrow(frame, p1, p2, color=(0, 0, 255))
                    drawn_edges.add(edge)

            else:
                print(f"Unknown action type: {action}")



        # Copy frame for this iteration so we can add text without modifying original
        frame_with_text = frame.copy()
        cv2.putText(frame_with_text, f"Iteration: {iteration}", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (30, 30, 30), 2)

        # draw solution path if last iteration
        if i == len(deltas):
            for j, node in enumerate(solution):
                if node in pos:
                    px, py = to_pixel(pos[node])
                    cv2.circle(frame_with_text, (px, py), 15, (0, 255, 0), -1)
                    # draw edge as line
                    if j < len(solution) - 1:
                        next_node = solution[j + 1]
                        if next_node in pos:
                            next_px, next_py = to_pixel(pos[next_node])
                            cv2.line(frame_with_text, (px, py), (next_px, next_py), (0, 255, 0), 10)


        cv2.imwrite(f"{output_dir}/step_{iteration:03d}.jpg", frame_with_text)

