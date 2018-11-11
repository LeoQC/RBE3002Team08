#!/usr/bin/env python

from simple_motion.srv import *
import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from turtlebot3_msgs.msg import SensorState
from tf.transformations import euler_from_quaternion


class Robot:

    def __init__(self):
        # initialize subscribers and publishers here as class variables
        # they can be used later in any other function
        """"
        Set up the node here
        """

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odoSub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.navtoSub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.nav_to_pose)
        # print 'hello world initted'
        rospy.init_node('SimpleMotion', anonymous=True)


        # rospy.spin()

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """
        print 'navigating to position'
        goalx = goal.pose.position.x
        goaly = goal.pose.position.y

        dx = goalx - self.x
        dy = goaly - self.y
        theta = math.atan2(dy,dx) #leave as is for first quadrant
        if theta>3.14:
            theta = -1*(2*3.14-theta)

        print 'dx: ', dx, ', dy: ', dy

        # theta = math.atan(dy/dx)
        distance = math.sqrt(math.pow(self.x-goalx,2) + math.pow(self.y-goaly,2))

        print 'current: ', self.x, ', ', self.y, ', angle: ', self.yaw
        print 'goal: ', goalx, ', ', goaly
        print 'theta: ', theta*180/3.14
        print 'distance: ', distance

        self.rotate(theta)
        print 'yaw: ', self.yaw
        self.drive_straight(0.25, distance)


    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive shraight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """


        print 'driving straight speed: ', speed, 'distance: ', distance
        twist = Twist()

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        startX = self.x
        startY = self.y

        traveled = 0.0;

        rate = rospy.Rate(10)
        while traveled < distance:
            traveled = math.sqrt(math.pow(self.x-startX,2) + math.pow(self.y-startY,2))

            if((twist.linear.x < speed) and (distance - traveled > 0.05)):
                twist.linear.x += 0.05

            print 'position: ', self.x, ', ', self.y
            self.pub.publish(twist)
            rate.sleep()

        print 'arrived'
        self.stop()


    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        twist = Twist()
        print 'turn to: ', angle

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        # startA = self.roll
        rotSpeed = 1
        curr = self.yaw
        if curr-angle<0:
            twist.angular.z = rotSpeed

        elif curr-angle>0:
            twist.angular.z = -1*rotSpeed

        print 'rotate ', angle
        self.pub.publish(twist)
        rate = rospy.Rate(10)
        while (not rospy.is_shutdown()) and abs(curr-angle)>0.1:
            self.pub.publish(twist)
            curr = self.yaw
            print 'yaw: ', self.yaw
            rate.sleep()

        self.stop()


    def odom_callback(self, data):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        # print 'odom callback'
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        quat = data.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)
        # print 'position: ', self.x , ', ', self.y
        # print 'orientation: ', self.roll, ', ', self.pitch, ', ', self.yaw

    def stop(self):
        twist = Twist()

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        self.pub.publish(twist)

if __name__ == '__main__':
    # print 'now'
    rob = Robot()
    # print 'hello world'
    try:
        cmd = sys.argv[1]
        if(cmd == 'd'):
            print 'drive straight'
            s = float(sys.argv[2])
            d = float(sys.argv[3])
            rob.drive_straight(s, d)
        elif(cmd == 'r'):
            print 'rotate'
            angle = float(sys.argv[2])
            rob.rotate(angle)
        elif(cmd == 'n'):
            print 'waiting for navigation instructions'
            rospy.spin()
        elif(cmd == 's'):
            rob.stop()
    except:
        print 'simple motion functions'
        print '-----------------------'
        print '\'d\' = drive straight: speed, distance'
        print '\'r\' = rotate: angle'
        print '\'n\' = rotate and drive to positions published on /move_base_simple/goal'