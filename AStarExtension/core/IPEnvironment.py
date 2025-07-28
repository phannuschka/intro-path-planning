# coding: utf-8

"""
This code is part of a series of notebooks regarding  "Introduction to robot path planning".

License is based on Creative Commons: Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) (pls. check: http://creativecommons.org/licenses/by-nc/4.0/)
"""
import cv2
import shapely

from IPPerfMonitor import IPPerfMonitor

import matplotlib.pyplot as plt

from shapely.geometry import Point, Polygon, LineString
from shapely import plotting
from shapely.strtree import STRtree

import numpy as np

class CollisionChecker(object):

    def __init__(self, scene, limits=[[0.0, 22.0], [0.0, 22.0]], statistic=None):
        self.scene = scene
        for key, value in self.scene.items():
            shapely.prepare(value)
        self.spatial_index = STRtree(list(self.scene.values()))
        self.limits = limits

    def getDim(self):
        """ Return dimension of Environment (Shapely should currently always be 2)"""
        return 2

    def getEnvironmentLimits(self):
        """ Return limits of Environment"""
        return list(self.limits)

    @IPPerfMonitor
    def pointInCollision(self, pos):
        """ Return whether a configuration is
        inCollision -> True
        Free -> False """
        assert (len(pos) == self.getDim())
        for key, value in self.scene.items():
            if value.intersects(Point(pos[0], pos[1])):
                return True
        return False

    @IPPerfMonitor
    def lineInCollision(self, startPos, endPos):
        """ Check whether a line from startPos to endPos is colliding"""
        assert (len(startPos) == self.getDim())
        assert (len(endPos) == self.getDim())
        
        p1 = np.array(startPos)
        p2 = np.array(endPos)
        p12 = p2-p1
        k = 40
        #print("testing")
        for i in range(k):
            testPoint = p1 + (i+1)/k*p12
            if self.pointInCollision(testPoint)==True:
                return True
        
        return False
                

#        for key, value in self.scene.items():
#            if value.intersects(LineString([(startPos[0], startPos[1]), (endPos[0], endPos[1])])):
 #               return True
#        return False

    def drawObstacles(self, ax):
        for key, value in self.scene.items():
            plotting.plot_polygon(value, add_points=False, color='red')


    def drawOnImage(self, img, scale, offset, obstacle_color=(100, 100, 255)):
        """Draw obstacles on OpenCV image

        Args:
            img: OpenCV image to draw on
            scale: scaling factor from world coordinates to image coordinates
            offset: (x_offset, y_offset) tuple for image positioning
            obstacle_color: BGR color tuple for obstacles (default red)

        Returns:
            img: The modified image
        """
        x_offset, y_offset = offset

        for key, value in self.scene.items():
            if hasattr(value, 'exterior'):
                # It's a Polygon
                coords = list(value.exterior.coords)

                # Convert world coordinates to image coordinates
                img_coords = []
                for x, y in coords:
                    img_x = int(x * scale + x_offset)
                    img_y = int(img.shape[0] - (y * scale + y_offset))
                    img_coords.append([img_x, img_y])

                # Draw filled polygon
                pts = np.array(img_coords, np.int32)
                cv2.fillPoly(img, [pts], obstacle_color)

            elif hasattr(value, 'coords'):
                # It might be a LineString or Point
                coords = list(value.coords)
                if len(coords) > 1:
                    # LineString - draw as connected lines
                    img_coords = []
                    for x, y in coords:
                        img_x = int(x * scale + x_offset)
                        img_y = int(img.shape[0] - (y * scale + y_offset))
                        img_coords.append((img_x, img_y))

                    # Draw lines connecting the points
                    for i in range(len(img_coords) - 1):
                        cv2.line(img, img_coords[i], img_coords[i + 1], obstacle_color, 2)

                elif len(coords) == 1:
                    # Point - draw as small circle
                    x, y = coords[0]
                    img_x = int(x * scale + x_offset)
                    img_y = int(img.shape[0] - (y * scale + y_offset))
                    cv2.circle(img, (img_x, img_y), 3, obstacle_color, -1)

        return img