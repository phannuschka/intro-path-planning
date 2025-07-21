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
import math
import numpy as np
from dataclasses import dataclass
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

        self.goal      =  []
        self.goalFound = False

        self.limits = self._collisionChecker.getEnvironmentLimits()

        self.dof = None
        self.discretization = None

        self.check_connection = False

        self.iteration = 0

        self.w = 0.5

        self.deltas = []

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
                'current_best': True,
                'collision': node_data['collision'],
                'line_collision': node_data['line_collision'],
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
            breakNumber = 0
            while currentBestName:
              # if breakNumber > 1000:
              #   break
              self.iteration = breakNumber
              breakNumber += 1

              currentBest = self.graph.nodes[currentBestName]

              if self._isNeighbour(currentBest["pos"], self.goal, discretization_steps):
                if not self._collisionChecker.lineInCollision(currentBest["pos"], self.goal):
                    self.solutionPath = []
                    self._addGraphNode(self.goal, currentBestName, name="goal")

                    self._collectPath( "goal", self.solutionPath )
                    self.goalFound = True
                    break

              currentBest["status"]= 'closed'
              if self._collisionChecker.pointInCollision(currentBest["pos"]):
                currentBest['collision']= 1
                currentBestName = self._getBestNodeName()

                if self.store_viz:
                    self._store_iteration_delta('close_node', currentBestName, currentBest)

                continue

              if self.check_connection:
                  fathers = list(self.graph.successors(currentBestName))
                  if len(fathers) == 1:
                      # Note: check the segment from currentBest to parent for collision
                      if self._collisionChecker.lineInCollision(currentBest["pos"], self.graph.nodes[fathers[0]]['pos']):
                            currentBest['line_collision'] = 1
                            self.graph.nodes[currentBestName]["g"] = float('inf')
                            print("line in collision, removing node:", currentBestName)

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
        # if fatherName is not None:
        #     if self.check_connection:
        #         # Note: check the segment from currentBest to parent for collision
        #         if self._collisionChecker.lineInCollision(pos, self.graph.nodes[fatherName]['pos']):
        #               return
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

        if replaceInOpen:
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

                    newNodeID = self._getNodeID(newPos)
                    if newNodeID in self.graph.nodes:
                        if self.reopen:
                            newPosOldG = self.graph.nodes[newNodeID]["g"]

                            node = self.graph.nodes[nodeName]
                            pos1 = np.array(node["pos"])
                            pos2 = np.array(newPos)
                            distance = np.linalg.norm(pos1 - pos2)

                            tentativeGScore = node["g"] + distance

                            if tentativeGScore < newPosOldG:
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

                                if tentativeGScore < newPosOldG:
                                    self.graph.remove_edges_from(list(self.graph.out_edges(newNodeID)))
                                    self._addGraphNode(newPos, nodeName, replaceInOpen=True)
                        else:
                            self._addGraphNode(newPos,nodeName)

        return result


    @IPPerfMonitor
    def _computeHeuristicValue(self, nodeName):
        """ Computes Heuristic Value: Manhattan Distance """

        result = 0
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
            break;
        return result


