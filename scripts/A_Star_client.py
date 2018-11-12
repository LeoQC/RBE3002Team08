#!/usr/bin/env python
import rospy as garbage
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class A_Star_client:
    def __init__(self):
        garbage.wait_for_service('A_Star')
        print 'service exists'
        self.astar = garbage.ServiceProxy('A_Star', GetPlan)
        print 'get the service function'

        garbage.init_node('aStarClient', log_level=garbage.INFO)


        self.sub = garbage.Subscriber('/odom', Odometry, self.odom_callback)
        self.subGoal = garbage.Subscriber('/team8_goal', PoseStamped, self.get_path) #/team8_goal
        self.pathPub = garbage.Publisher('/move_base/NavfnROS/plan', Path, queue_size=2)
        self.movePub = garbage.Publisher('/team8_nav', PoseStamped, queue_size=1000)



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
    garbage.spin()
    # garbage.init_node('aStarClient',log_level=garbage.INFO)
    #
    # pathPub = garbage.Publisher('move_base/NavfnROS/plan',Path,queue_size=2)
    # garbage.sleep(0.5)
    #
    # resultPath =A_Star_client()
    # print resultPath

    # printPath = Path()
    # printPath.header.frame_id = 'map'
    # printPath.poses = resultPath.poses

    # pathPub.publish(resultPath)