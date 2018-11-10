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


def map_to_world(x, y, my_map):

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

def printEmptyPath ():
	EmptyPath = Path()
	EmptyPath.header.frame_id = 'map'
	pathPub.publish(EmptyPath)



if __name__ == "__main__":

	rospy.init_node('printingshit',log_level=rospy.INFO)

	pathPub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=2)
	rospy.sleep(0.3) # needed for things to be able to publish 


	my_map = GetMap() # setup type
	my_map = dynamic_map_client()


	printEmptyPath()
	
	testingMap_to_world(my_map)



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
