# coding: utf-8
"""
This code is part of the course "Introduction to robot path planning" (Author: Bjoern Hein).

It is based on the slides given during the course, so please **read the information in theses slides first**

Remark: The code is a slightly modified version of AStar, whithout reopening of nodes, when once in the closed list.

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""

from typing import List
import copy
import networkx as nx
import heapq
import numpy as np

from scipy.spatial.distance import euclidean, cityblock

from IPPlanerBase import PlanerBase
from IPPerfMonitor import IPPerfMonitor


class AStar(PlanerBase):
    def __init__(self, collChecker = 0):
        """Contructor:

        Initialize all necessary members"""

        super(AStar,self).__init__(collChecker)
        self.graph = nx.DiGraph() # = CloseList
        self.openList = [] # (<value>, <node>)

        self.start = []
        self.goal      =  []
        self.goalFound = False
        self.solutionPath = []

        self.limits = self._collisionChecker.getEnvironmentLimits()

        self.dof = None
        self.discretization = None
        self.discretization_steps = None

        self.check_connection = False
        self.lazy_check_connection = False
        self.reopen = False

        self.iteration = 0

        self.w = 0.5
        self.heuristic = None

        self.deltas = []  # for visualization purposes
        self.store_viz = False

        return

    def _store_iteration_delta(self, action_type, node_name=None, node_data=None, edge_data=None):
        """Store the iteration delta for visualization purposes."""
        delta = {
            'iteration': self.iteration,
            'action_type': action_type,
        }
        if action_type == 'add_node':
            delta.update({
                'node_name': node_name,
                'g': node_data['g'],
                'status': node_data['status'],
            })
        elif action_type == 'update_node':
            delta.update({
                'node_name': node_name,
                'g': node_data['g'],
                'status': node_data['status'],
                'collision': node_data['collision'],
                'line_collision': node_data['line_collision'],
            })
        elif action_type == 'close_node':
            delta.update({
                'node_name': node_name,
                'collision': node_data['collision'],
                'line_collision': node_data['line_collision'],
                'status': node_data['status'],
            })
        elif action_type == 'add_edge':
            delta.update({
                'from_node': edge_data['from_node'],
                'to_node': edge_data['to_node'],
            })
        elif action_type == 'remove_edge':
            delta.update({
                'from_node': edge_data['from_node'],
                'to_node': edge_data['to_node'],
            })

        self.deltas.append(delta)

    def _getNodeID(self, pos):
        """Compute a unique identifier based on the position"""
        # we cannot take the position directly as node name, because of floating point precision issues
        # instead we discretize the position based on the discretization steps
        pos_a = np.array(pos)
        start = np.array(self.start)
        pos_a = pos_a - start
        pos_a = np.round(pos_a / np.array(self.discretization_steps)).astype(int)

        nodeId = "-"
        for i in pos_a:
            nodeId += str(i)+"-"
        return nodeId

    @IPPerfMonitor
    def planPath(self, startList, goalList, config, store_viz=False):
        """

        Args:
            start (array): start position in planning space
            goal (array) : goal position in planning space
            config (dict): dictionary with the needed information about the configuration options

        Example:

            config["w"] = 0.5
            config["heuristic"] = "euclid"

        """
        self.dof = config["dof"]
        self.discretization = config["discretization"]
        self.reopen = config["reopen"]
        self._setLimits(config["lowLimits"], config["highLimits"])
        self.check_connection = config["check_connection"]
        self.lazy_check_connection = config.get("lazy_check_connection", False)
        self.store_viz = store_viz

        # compute discretization steps for each dim
        discretization_steps = [0] * self.dof
        self.discretization_steps = discretization_steps
        for i in range(self.dof):
            axis_length = self.limits[i][1] - self.limits[i][0]
            discretization_steps[i] = axis_length / self.discretization[i]

        # 0. reset
        self.graph.clear()

        try:
            # 1. check start and goal whether collision free (s. BaseClass)
            checkedStartList, checkedGoalList = self._checkStartGoal(startList,goalList)

            # 2.
            self.w = config["w"]
            self.heuristic = config["heuristic"]

            self.goal = checkedGoalList[0]
            self.start = checkedStartList[0]
            self._addGraphNode(checkedStartList[0])

            currentBestName = self._getBestNodeName()
            self.iteration = 0
            while currentBestName:
              self.iteration += 1

              currentBest = self.graph.nodes[currentBestName]

              # Note: we start the grid search from start position,s o it is always included in the graph
              # for goal, we check whether it is a neighbour (closer than a discretization step) of the current best node
              if self._isNeighbour(currentBest["pos"], self.goal, discretization_steps):
                if not self._collisionChecker.lineInCollision(currentBest["pos"], self.goal):
                    self._addGraphNode(self.goal, currentBestName, name="goal")
                    self._collectPath( "goal", self.solutionPath )
                    self.goalFound = True
                    break

              currentBest["status"]= 'closed'
              if self._collisionChecker.pointInCollision(currentBest["pos"]):
                currentBest['collision']= 1
                if self.store_viz:
                    self._store_iteration_delta('close_node', currentBestName, currentBest)
                currentBestName = self._getBestNodeName()
                continue

              # Note: check the segment from currentBest to parent for collision
              # doing it this late looses optimality, even with reopening
              if self.check_connection and self.lazy_check_connection:
                  fathers = list(self.graph.successors(currentBestName))
                  if len(fathers) == 1:
                      if self._collisionChecker.lineInCollision(currentBest["pos"], self.graph.nodes[fathers[0]]['pos']):
                            currentBest['line_collision'] = 1
                            self.graph.nodes[currentBestName]["g"] = float('inf')
                            if self.store_viz:
                                self._store_iteration_delta('close_node', currentBestName, currentBest)
                            currentBestName = self._getBestNodeName()
                            continue

              self.graph.nodes[currentBestName]['collision'] = 0

              if self.store_viz:
                self._store_iteration_delta('close_node', currentBestName, currentBest)

              # handleNode merges with former expandNode
              self._handleNode(currentBestName, discretization_steps)
              currentBestName = self._getBestNodeName()

            if self.goalFound:
                return self.solutionPath, self.deltas
            else:
                return None, []
        except Exception as e:
            print("Planning failed:", e)
            return None, []

    def _isNeighbour(self, pos1, pos2, discretization_steps: List[float]) -> bool:
        """Check whether two positions are within 1 discretization step in each dimension"""
        assert(len(pos1) == len(pos2) == self.dof)
        for i in range(self.dof):
            if abs(pos1[i] - pos2[i]) > discretization_steps[i]:
                return False
        return True

    def _insertNodeNameInOpenList(self, nodeName):
        """Get an existing node stored in graph and put it in the OpenList"""
        heapq.heappush(self.openList,(self._evaluateNode(nodeName),nodeName))

    @IPPerfMonitor
    def _addGraphNode(self, pos, fatherName=None, replaceInOpen=False, name=None):
        """Add a node based on the position into the graph. Attention: Existing node is overwritten!"""
        nodeName = name if name is not None else self._getNodeID(pos)
        self.graph.add_node(nodeName, pos=pos, status='open', g=0, collision=0, line_collision=0)

        if self.store_viz:
            self._store_iteration_delta('add_node', node_name=nodeName, node_data=self.graph.nodes[nodeName])


        if fatherName != None:
            self.graph.add_edge(nodeName, fatherName)
            pos1 = np.array(pos)
            pos2 = np.array(self.graph.nodes[fatherName]["pos"])
            distance = np.linalg.norm(pos1 - pos2)
            self.graph.nodes[nodeName]["g"] = self.graph.nodes[fatherName]["g"] + distance

            if self.store_viz:
                self._store_iteration_delta('add_edge', edge_data={'from_node': fatherName, 'to_node': nodeName})
                self._store_iteration_delta('update_node', node_name=nodeName, node_data=self.graph.nodes[nodeName])

        if replaceInOpen:  # for reopening we have to remove the node from the open list to avoid duplicates
            keys = list(map(lambda x: x[1], self.openList))
            if nodeName in keys:
                idx = keys.index(nodeName)
                del self.openList[idx]

        self._insertNodeNameInOpenList(nodeName)

   
    def _setLimits(self, lowLimit, highLimit):
        """ Sets the limits of the investigated search space """
        assert(len(lowLimit)==len(highLimit)==self.dof)
        self.limits = list()
        for i in range(self.dof):
          self.limits.append([lowLimit[i],highLimit[i]])
        return
  
    def _getBestNodeName(self):
        """ Returns the best name of best node """
        return heapq.heappop(self.openList)[1]

    @IPPerfMonitor
    def _handleNode(self, nodeName, discretization_steps: List[float]):
        """Generats possible successor positions in all dimensions"""
        result =  []
        node = self.graph.nodes[nodeName]
        for i in range(len(node["pos"])):
            discretization_step = discretization_steps[i]
            for u in [-discretization_step, discretization_step]:
                    newPos = copy.copy(node["pos"])
                    newPos[i] += u
                    if not self._inLimits(newPos):
                        continue

                    if self.check_connection and not self.lazy_check_connection:  # do not add nodes not reachable from the current node
                        if self._collisionChecker.lineInCollision(node["pos"], newPos):
                            continue

                    newNodeID = self._getNodeID(newPos)
                    if newNodeID in self.graph.nodes:
                        if self.reopen:
                            newPosOldG = self.graph.nodes[newNodeID]["g"]

                            node = self.graph.nodes[nodeName]
                            tentativeGScore = node["g"] + discretization_step

                            # reopening of nodes if the new g value is smaller than the old one (avoid numerical issues)
                            if tentativeGScore < newPosOldG and abs(tentativeGScore - newPosOldG) > 0.5 * discretization_step:
                                print("reopening node:", newNodeID, "with g=", tentativeGScore, "old g=", newPosOldG)

                                if self.store_viz:
                                    edges = self.graph.out_edges(newNodeID)
                                    if len(edges) > 0:
                                        for edge in edges:
                                            self._store_iteration_delta('remove_edge', edge_data={'from_node': edge[1], 'to_node': edge[0]})
                                self.graph.remove_edges_from(list(self.graph.out_edges(newNodeID)))
                                self._addGraphNode(newPos,nodeName, replaceInOpen=True)
                    else:
                        self._addGraphNode(newPos,nodeName)
        return result

    @IPPerfMonitor
    def _handleNode9(self, nodeName, discretization_steps: List[float]):
        """Generats possible successor positions also in diagonal direction"""
        result =  []
        node = self.graph.nodes[nodeName]
        for i in range(len(node["pos"])):
          for j in range(len(node["pos"])):
              if i == j:
                  continue
              discretization_step_u = discretization_steps[i]
              discretization_step_v = discretization_steps[j]
              for u in [-discretization_step_u, discretization_step_u]:
                  for v in [-discretization_step_v, 0, discretization_step_v]:
                        newPos = copy.copy(node["pos"])
                        newPos[i] += u
                        newPos[j] += v
                        if not self._inLimits(newPos):
                            continue
                        
                        newNodeID = self._getNodeID(newPos)
                        if newNodeID in self.graph.nodes:
                            if self.reopen:
                                newPosOldG = self.graph.nodes[newNodeID]["g"]
                                
                                node = self.graph.nodes[nodeName]
                                pos1 = np.array(node["pos"])
                                pos2 = np.array(newPos)
                                distance = np.linalg.norm(pos1 - pos2)

                                tentativeGScore = node["g"] + distance

                                # reopening of nodes if the new g value is smaller than the old one (avoid numerical issues)
                                if tentativeGScore < newPosOldG and (
                                        abs(tentativeGScore - newPosOldG) > 0.5 * discretization_step_u or
                                        abs(tentativeGScore - newPosOldG) > 0.5 * discretization_step_v):
                                    print("reopening node:", newNodeID, "with g=", tentativeGScore, "old g=", newPosOldG)

                                    if self.store_viz:
                                        edges = self.graph.out_edges(newNodeID)
                                        if len(edges) > 0:
                                            for edge in edges:
                                                self._store_iteration_delta('remove_edge',
                                                                            edge_data={'from_node': edge[1],
                                                                                       'to_node': edge[0]})
                                    self.graph.remove_edges_from(list(self.graph.out_edges(newNodeID)))
                                    self._addGraphNode(newPos, nodeName, replaceInOpen=True)
                        else:
                            self._addGraphNode(newPos,nodeName)

        return result


    @IPPerfMonitor
    def _computeHeuristicValue(self, nodeName):
        """ Computes Heuristic Value: Eucclidean or Manhattan Distance """
        node = self.graph.nodes[nodeName]
        if self.heuristic == "euclidean":
            return euclidean(self.goal, node["pos"])
        else:
            return cityblock(self.goal, node["pos"])

    @IPPerfMonitor
    def _evaluateNode(self, nodeName):
        node = self.graph.nodes[nodeName]
        return  self.w * self._computeHeuristicValue(nodeName) + (1 - self.w) * node["g"]

                      
    def _collectPath(self, nodeName, solutionPath):
      fathers = list(self.graph.successors(nodeName))
      if len(fathers) == 1:
        self._collectPath( fathers[0], solutionPath )
      elif len(fathers) == 0:
        solutionPath.append(nodeName)
        return
      else:
        raise Exception("not suitable numbers of fathers = {}.... please check".format(len(fathers)))
      solutionPath.append(nodeName)
      return
  
    @IPPerfMonitor
    def _inLimits(self, pos):
        result = True
        for i, limit in enumerate(self.limits):
          if pos[i] < limit[0] or pos[i] > limit[1]:
            result = False
            break
        return result


