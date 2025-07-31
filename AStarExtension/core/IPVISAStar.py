# coding: utf-8

"""
This code is part of the course 'Innovative Programmiermethoden für Industrieroboter' (Author: Bjoern Hein). It is based on the slides given during the course, so please **read the information in theses slides first**

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""
import time

import networkx as nx
import matplotlib.pyplot as plt
import os
import cv2
import numpy as np


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

