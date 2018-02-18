""" An implementation of an occupancy field that you can use to implement
    your particle filter's laser_update function """

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors

class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
        obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
    """

    def __init__(self, map):
        self.map = map      # save this for later
        self.free_cells = []
        # build up a numpy array of the coordinates of each grid cell in the map
        X = np.zeros((self.map.info.width*self.map.info.height,2))

        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                ind = i + j*self.map.info.width
                if self.map.data[ind] == 0:
                    self.free_cells.append({
                        'x': float(i)*self.map.info.resolution + self.map.info.origin.position.x,
                        'y': float(j) * self.map.info.resolution + self.map.info.origin.position.y,
                    })
                if self.map.data[ind] > 0:
                    total_occupied += 1
                X[curr,0] = float(i)
                X[curr,1] = float(j)
                curr += 1

        # build up a numpy array of the coordinates of each occupied grid cell in the map
        O = np.zeros((total_occupied,2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order, if you go through this right, you might be able to use curr
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    O[curr,0] = float(i)
                    O[curr,1] = float(j)
                    curr += 1

        # use super fast scikit learn nearest neighbor algorithm
        nbrs = NearestNeighbors(n_neighbors=1,algorithm="ball_tree").fit(O)
        distances, indices = nbrs.kneighbors(X)

        self.closest_occ = {}
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                ind = i + j*self.map.info.width
                self.closest_occ[ind] = distances[curr][0]*self.map.info.resolution
                curr += 1

    def get_closest_obstacle_distance(self,x,y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
            is out of the map boundaries, nan will be returned. """
        if math.isnan(x) or math.isnan(y):
            return float('nan')

        x_coord = int((x - self.map.info.origin.position.x)/self.map.info.resolution)
        y_coord = int((y - self.map.info.origin.position.y)/self.map.info.resolution)

        # check if we are in bounds
        if x_coord > self.map.info.width or x_coord < 0:
            return float('nan')
        if y_coord > self.map.info.height or y_coord < 0:
            return float('nan')

        ind = x_coord + y_coord*self.map.info.width
        if ind >= self.map.info.width*self.map.info.height or ind < 0:
            return float('nan')
        return self.closest_occ[ind]
