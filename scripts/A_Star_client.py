#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def A_Star_client():
    rospy.wait_for_service('A_Star')
    print 'service exists'
    astar = rospy.ServiceProxy('A_Star', GetPlan)
    print 'get the service function'

    StartPoint = PoseStamped()
    StartPoint.pose.position.x=0
    StartPoint.pose.position.y=0
    EndPoint = PoseStamped()
    EndPoint.pose.position.x=-3
    EndPoint.pose.position.y=1.5

    response = astar(StartPoint,EndPoint,0)
    return response

if __name__ == "__main__":

    rospy.init_node('aStarClient',log_level=rospy.INFO)

    pathPub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=2)
    rospy.sleep(0.5)

    resultPath =A_Star_client()
    print resultPath
    pathPub.publish(resultPath)