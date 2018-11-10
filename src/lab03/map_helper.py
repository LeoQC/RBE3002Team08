#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """

    neighbors = []
    if (loc[0] >= 0 and loc[0] < my_map.info.height) and (loc[1] >= 0 and loc[1] < my_map.info.width):
        width = my_map.info.width
        height = my_map.info.height

        # x+1,y
        if ((loc[0] + 1) < height) and (my_map.data[loc[0] + 1 + (loc[1]*width)] != 100):
            neighbors.append((loc[0] + 1, loc[1]))
        # x,y+1
        if ((loc[1] + width) < width) and (my_map.data[loc[0] + ((loc[1]+1)*width)] != 100):
            neighbors.append((loc[0], loc[1] + 1))
        # x,y-1
        if ((loc[1] - width) > 0) and (my_map.data[loc[0] + ((loc[1]-1)*width)] != 100):
            neighbors.append((loc[0], loc[1] - 1))
        # x-1,y
        if ((loc[0] - 1) > 0) and (my_map.data[loc[0] - 1 + (loc[1]*width)] != 100):
            neighbors.append((loc[0] - 1, loc[1]))
    else:
        print("Cell not valid")

    return neighbors

def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """


  


def convert_location(loc, my_map):
    """converts points to the grid"""
   

def world_to_map(x, y, my_map):
    """
        converts a point from the world to the map
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """
    xOff = my_map.info.origin.position.x
    yOff = my_map.info.origin.position.y
    scale = my_map.info.resolution
    xMap = (x - xOff) / scale
    yMap = (y - yOff) / scale
    xMap = int(xMap)
    yMap = int(yMap)
    return (xMap,yMap)

def map_to_world(x, y, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """
    xOff = my_map.info.origin.position.x
    yOff = my_map.info.origin.position.y
    scale = my_map.info.resolution
    xWorld = x * scale + xOff  
    yWorld = y * scale + yOff
    return (xWorld,yWorld)


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """


def index_to_point(point, my_map):
    """convert a point to a index"""

def point_to_index(location, my_map):
    """convert a index to a point"""
