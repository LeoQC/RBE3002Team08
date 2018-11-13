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

    x = loc[0]
    y = loc[1]
    width = my_map.info.width
    height = my_map.info.height

    if (x >= 0 and x < height) and (y >= 0 and y < width):

        # x+1,y
        if ((x + 1) < height) and (my_map.data[x + 1 + (y*width)] != 100):
            neighbors.append((x + 1, y))
        # x,y+1
        if ((y + 1) < width) and (my_map.data[x + ((y+1)*width)] != 100):
            neighbors.append((x, y + 1))
        # x,y-1
        if ((y - 1) > 0) and (my_map.data[x + ((y-1)*width)] != 100):
            neighbors.append((x, y - 1))
        # x-1,y
        if ((x - 1) > 0) and (my_map.data[x - 1 + (y*width)] != 100):
            neighbors.append((x - 1, y))
    else:
        print("Cell not valid")

    return neighbors


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
    return (xMap, yMap)


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
    xWorld = (x+0.5) * scale + xOff  
    yWorld = (y+0.5) * scale + yOff
    return (xWorld, yWorld)
