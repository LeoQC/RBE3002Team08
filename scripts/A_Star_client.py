#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def A_Star_client():
    rospy.wait_for_service('A_Star')
    print 'service exists'
    astar = rospy.ServiceProxy('A_Star', GetPlan)
    print 'a thing happened here'
    response = astar()
    return response

if __name__ == "__main__":
    print A_Star_client()