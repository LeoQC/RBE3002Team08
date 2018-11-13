#!/usr/bin/env python

# by Leo Chen. non blocking robot controller

import rospy
import math
from geometry_msgs.msg import Twist, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

from nav_msgs.msg import Path



class Robot:

	# data classes. 

    class PoseStruct:
        x= None
        y = None
        heading = None 
        seq = -1 
        def __init__(self,x,y,heading):
            self.x=x
            self.y=y
            self.heading = heading
        
    class ValCmdStruct :
        def __init__(self,pub):
            self.pub = pub
            self.msg = Twist()
            self.sendTime = None
        def push(self):
            self.pub.publish(self.msg)
            self.sendTime =rospy.get_time()
        def setZ(self , speed):
            self.msg.angular.z = speed 
        def setX(self, speed):
            self.msg.linear.x=speed 



    def __init__(self):
        # setup subscribers and publishers
        # setup class variables to store things 
        print "start of init "
        
        self.odomSUb = rospy.Subscriber('odom', Odometry, self.odom_callback)
        print "setup odem "

        self.plan = [ self.PoseStruct(0,0,0)]
        self.planSeq=-1
        
        self.current = 	self.PoseStruct(0,0,0)
        self.desire =	self.PoseStruct(0,0,0)
        self.tolerance= self.PoseStruct(0.07,0.07,math.pi/6)
        self.valCmd = 	self.ValCmdStruct(rospy.Publisher('cmd_vel', Twist, queue_size=1))
        
        self.navtoSub = rospy.Subscriber('move_base/NavfnROS/plan', Path,self. planParser_callback)
        while self.current.seq <0 and self.current.seq<0 :
            pass
        print "got a odem position"
        print "end of init"

        # the core controller
        # this is designed to be run once until robot is at the goal
    def tracePath(self):
        pass 

        self.desire.seq = self.planSeq #  mark which plan is it tracing
        for point in self.plan:
            self.desire.x = point.x
            self.desire.y = point.y
            # keep moving until the next way point is reached
            while self.reachCheck() == False :
                xDiff = self.desire.x - self.current.x
                yDiff = self.desire.y - self.current.y
                self.desire.heading = math.atan2(yDiff,xDiff)
                # control loops 
                self.rotateControl()
                if abs(self.desire.heading) < self.tolerance.heading:
                    self.valCmd.msg.linear.x =0
                    self.valCmd.push()
                else:
                    self.linearControl()
            # reached the point, going to the next one
            self.stopMoving()
            print "reached the point, go to next one"

            if self.desire.seq != self.planSeq :
                self.stopMoving()
                print " plan is changed ! "
                return 
        print " done with the current path"

    def linearControl(self):
        pass
        return True

    def rotateControl(self):
        pass
        self.desire.heading
        return True

    def odom_callback(self,data):
        """
        update the state of the robot
        :type msg: Odom
        """
        self.current.x = data.pose.pose.position.x
        self.current.y = data.pose.pose.position.y
        quart = data.pose.pose.orientation 
        qq = [quart.x, quart.y, quart.z, quart.w] 
        self.current.heading = euler_from_quaternion(qq)  
        self.current.seq = data.header.seq

    def planParser_callback(self,plan):
        # param plan: Path
        self.planSeq = plan.header.seq
        self.plan=[]
        for point in plan.poses:
            pos = self.PoseStruct(point.pose.position.x,point.pose.position.y,0)
            self.plan.append(pos)
            print pos 
        print " in plan parser "
        
    def angleFix (self,angle):
        # change the input angle to within the -pi to pi range. 
        # type: msg: number in rad
        if angle <-math.pi:
            angle += math.pi*2 
        if angle > math.pi:
            angle -= math.pi*2
        return angle

    def reachCheck(self):
        if  abs(self.desire.x - self.current.x) < self.tolerance.x : 
            if  abs(self.desire.y - self.current.y) < self.tolerance.y : 
                return True
        return False

    def stopMoving(self):
        self.valCmd.msg.angular.z=0
        self.valCmd.msg.linear.x =0
        self.valCmd.push()


if __name__ == '__main__' : 

    # rospy.init_node('lab2Goal',log_level=rospy.DEBUG)
    rospy.init_node('complexMotion',log_level=rospy.INFO)
    R = Robot()

    while rospy.is_shutdown() == False:
        # if the robot havn't run though the given path, trace once. 
        if (R.desire.seq!= R.planSeq): 
            R.tracePath()
        else:
            pass 
    

    rospy.spin()


