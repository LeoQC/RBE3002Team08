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
from nav_msgs.msg import OccupancyGrid
# for displaying path  
from nav_msgs.msg import Path

def printingMap(map):
	width = map.info.width
	print " width is :", width
	
	output = "->"
	for cell in range (0,width):
		content = map.data[cell]
		if (content == 0 ):
			output +="O"
		else:
			output+="+"
	print output

	for item in range(0,100): print map.data[item]
	


if __name__ == "__main__":

	rospy.init_node('printingshit',log_level=rospy.INFO)

	pathLength = 10 
	
	navPath = Path()
	EmptyPath = Path()

	posList = []
	
	ii=0
	for ii in range(0,pathLength):
		pos = PoseStamped()
		pos.pose.position.x = 0 
		pos.pose.position.y = ii
		posList.append(pos)
	navPath.header.frame_id = 'map'
	# print(posList)
	navPath.poses=posList	

	pub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=1)
	
	rospy.Subscriber("map",OccupancyGrid,printingMap)


	for j in range(5): 
		print "I bloody did something"
		pub.publish(navPath)
		# pub.publish(EmptyPath)
		print "I did something else!"
		rospy.sleep(2)
		if rospy.is_shutdown() :
			break
