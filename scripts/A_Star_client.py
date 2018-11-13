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


        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.subGoal = rospy.Subscriber('/team8_goal', PoseStamped, self.get_path)
        self.pathPub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=2)
        self.movePub = rospy.Publisher('/team8_nav', PoseStamped, queue_size=1000)

    def get_path(self, goal):
        xTarget = goal.pose.position.x
        yTarget = goal.pose.position.y

        startPoint = PoseStamped()
        startPoint.pose.position.x = self.currPos[0]
        startPoint.pose.position.y = self.currPos[1]
        endPoint = PoseStamped()
        endPoint.pose.position.x = xTarget
        endPoint.pose.position.y = yTarget

        response = self.astar(startPoint, endPoint, 0)
        self.pathPub.publish(response.plan)

        self.follow_path(response.plan)

    def follow_path(self, path):

        for pose in path.poses:
            self.movePub.publish(pose)

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
