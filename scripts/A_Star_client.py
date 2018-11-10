#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan, Path
from geometry_msgs.msg import PoseStamped

def A_Star_client():
    rospy.wait_for_service('A_Star')

    astar = rospy.ServiceProxy('A_Star', GetPlan)
    response = astar()
    # return response.plan

if __name__ == "__main__":
