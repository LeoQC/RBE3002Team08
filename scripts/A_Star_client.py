#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class A_Star_client:
    def __init__(self):
        rospy.wait_for_service('A_Star')
        print 'service exists'
        self.astar = rospy.ServiceProxy('A_Star', GetPlan)
        print 'get the service function'

        rospy.init_node('aStarClient', log_level=rospy.INFO)

        self.sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.subGoal = rospy.Subscriber("/team8_goal", PoseStamped, self.get_path)
        self.pathPub = rospy.Publisher('move_base/NavfnROS/plan', Path, queue_size=2)


        # StartPoint = PoseStamped()
        # StartPoint.pose.position.x=0
        # StartPoint.pose.position.y=0
        # EndPoint = PoseStamped()
        # EndPoint.pose.position.x=-2
        # EndPoint.pose.position.y=-2
        #
        # response = astar(StartPoint, EndPoint, 0)

        # return response.plan

    def get_path(self, goal):
        xTarget = goal.pose.position.x
        yTarget = goal.pose.position.y
        # quat = goal.pose.orientation
        # q = [quat.x, quat.y, quat.z, quat.w]
        # roll, pitch, yaw = euler_from_quaternion(q)

        StartPoint = PoseStamped()
        StartPoint.pose.position.x = self.currPos[0]
        StartPoint.pose.position.y = self.currPos[1]
        EndPoint = PoseStamped()
        EndPoint.pose.position.x = xTarget
        EndPoint.pose.position.y = yTarget

        response = self.astar(StartPoint, EndPoint, 0)
        self.pathPub.publish(response.plan)



    def odom_callback(self, msg):

        """
        update the state of the robot
        :type msg: Odom
        """

        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.currPos = [px, py, roll, pitch, yaw]

if __name__ == "__main__":

    astar = A_Star_client()
    rospy.spin()
    # rospy.init_node('aStarClient',log_level=rospy.INFO)
    #
    # pathPub = rospy.Publisher('move_base/NavfnROS/plan',Path,queue_size=2)
    # rospy.sleep(0.5)
    #
    # resultPath =A_Star_client()
    # print resultPath

    # printPath = Path()
    # printPath.header.frame_id = 'map'
    # printPath.poses = resultPath.poses

    # pathPub.publish(resultPath)