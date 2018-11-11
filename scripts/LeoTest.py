#!/usr/bin/env python

import copy
import math 
import numpy as np
import rospy
import tf 
import string 

from geometry_msgs.msg import (PoseStamped, Twist, Pose )
# somehow these line import fine but can't do rospack find Pose 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from std_msgs.msg import Empty

# for read the maze
# for displaying path  
from nav_msgs.msg import Path


from nav_msgs.srv import GetMap


def dynamic_map_client():

	"""
		Service call to get map and set class variables
		This can be changed to call the expanded map
		:return:
	"""
	# call nav_msgs/GetMap

	rospy.wait_for_service('static_map') 
	try:
		currentMap = rospy.ServiceProxy('static_map', GetMap)
		resp = currentMap()
		# print resp
		return resp.map
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def printEmptyPath ():
	EmptyPath = Path()
	EmptyPath.header.frame_id = 'map'
	pathPub.publish(EmptyPath)



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

def testingMap_to_world(my_map):
	
	# make a path in world from cell 0,0 to cell 5,5 
	mapPoints = [(0,0),(1,1),(2,2),(3,3),(4,3),(5,3) ] # the index coords in maps
	printPath = Path()
	printPath.header.frame_id = 'map'
	posList = []
	for mapPoint in mapPoints:
		worldPoint = map_to_world(mapPoint[0],mapPoint[1],my_map)
		print "world Point at! :" , worldPoint
		pos = PoseStamped()
		pos.pose.position.x = worldPoint[0]
		pos.pose.position.y = worldPoint[1]
		posList.append(pos)
	printPath.poses=posList	
	pathPub.publish(printPath)

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

def testingWorld_to_map(my_map):
	
	# make a path in world from cell 0,0 to cell 5,5 
	worldPoints = [(-5,-5),(0,0),(-5,0),(-4,1),(-3,1 ),(-3,2),(0,2) ] # the index coords in maps

	# map this back to map space
	mapPoints =[]
	for point in worldPoints:
		mapPoint = world_to_map(point[0],point[1],my_map)
		print "point on Map: " ,mapPoint
		mapPoints.append(mapPoint)


	printPath = Path()
	printPath.header.frame_id = 'map'
	posList = []
	for mapPoint in mapPoints:
		printPoint = map_to_world(mapPoint[0],mapPoint[1],my_map)
		print "world Point at! :" , printPoint
		pos = PoseStamped()
		pos.pose.position.x = printPoint[0]
		pos.pose.position.y = printPoint[1]
		posList.append(pos)
	printPath.poses=posList	
	pathPub.publish(printPath)

def euclidean_heuristic(point1, point2):
	"""
		calculate the dist between two points
		:param point1: tuple of location
		:param point2: tuple of location
		:return: dist between two points
	"""
	xDiff = point1[0] - point2[0] 
	yDiff = point1[1] - point2[1]
	dis = math.sqrt(xDiff*xDiff + yDiff*yDiff)
	return dis

def TestEuclidean_heuristic():
	point1 = (0,0)
	point2 = (3,4)
	dis = euclidean_heuristic(point1,point2)
	print "TestEuclidean_heuristic"
	print "Diss between: ",point1, point2," is : ",dis

def optimize_path( path):
	"""
		remove redundant points in hte path
		:param path: list of tuples
		:return: reduced list of tuples
	"""
	optPath = [path[0]]

	optPath[-1]
	# ii=1
	# current[ii]
	



if __name__ == "__main__":

	rospy.init_node('printingshit',log_level=rospy.INFO)

	pathPub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=2)
	rospy.sleep(0.3) # needed for things to be able to publish 


	my_map = GetMap() # setup type
	my_map = dynamic_map_client()


	printEmptyPath()
	
	# testingMap_to_world(my_map)
	# testingWorld_to_map(my_map)
	
	TestEuclidean_heuristic()


#  constructing a path 

	# pathLength = 10 
	# navPath = Path()
	# posList = []
	# ii=0
	# for ii in range(0,pathLength):
	# 	pos = PoseStamped()
	# 	pos.pose.position.x = 0 
	# 	pos.pose.position.y = ii/2
	# 	posList.append(pos)
	# navPath.header.frame_id = 'map'
	# # print(posList)
	# navPath.poses=posList	

	# pub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=1)
	


# printing this path

	# for j in range(1): 
	# 	print "I bloody did something"
	# 	pub.publish(navPath)
	# 	# pub.publish(EmptyPath)
	# 	print "I did something else!"
	# 	rospy.sleep(0.5)
	# 	if rospy.is_shutdown() :
	# 		break
