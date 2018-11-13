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
        self.desire =	self.PoseStruct(0,0,0.0)
        self.tolerance= self.PoseStruct(0.01,0.01,math.pi/10)
        self.valCmd = 	self.ValCmdStruct(rospy.Publisher('cmd_vel', Twist, queue_size=1))
        self.navtoSub = rospy.Subscriber('move_base/NavfnROS/plan', Path,self. planParser_callback)
        while self.current.seq <0 and self.current.seq<0 :
            pass
        print "got a odem position"
        print "end of init"

        # the core controller
        # this is designed to be run once until robot is at the goal
    def tracePath(self):
        print " start tracing with seq of: " , self.planSeq
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
                if abs(self.desire.heading - self.current.heading) >  self.tolerance.heading:
                    self.valCmd.msg.linear.x = 0
                    self.valCmd.push()
                else:
                    self.linearControl()


                if rospy.is_shutdown() == True :
                    self.stopMoving()
                    return
            # reached the point, going to the next one
            self.stopMoving()
            print "reached the point, go to next one"

            if self.desire.seq != self.planSeq :
                self.stopMoving()
                print " plan is changed ! "
                return 
        print " done with the current path"

    def linearControl(self):
        xDiff = self.desire.x - self.current.x
        yDiff = self.desire.y - self.current.y
        dis = math.sqrt( xDiff*xDiff + yDiff*yDiff)  
        pGain = 1      
        moveSpeed = pGain*dis
        if moveSpeed>0.8 :
            moveSpeed = 1
        if moveSpeed<0.3 :
            moveSpeed = 0.3
        self.valCmd.setX(moveSpeed)
        self.valCmd.push()

        return False 

    def rotateControl(self):
        pGain = 1.2
        angleDiff = self.desire.heading - self.current.heading   # calculate the amount of turnning needed
        angleDiff = self.angleFix(angleDiff)            # fit them within -180 to 180 
        turnSpeed = angleDiff * pGain    
        if abs(turnSpeed)>1.4 :
            turnSpeed = math.copysign(1.4,turnSpeed)
        if abs(turnSpeed)<0.4 :
            turnSpeed = math.copysign(0.4,turnSpeed)

        print "self,desire,diff : %.3f  %.3f  %.3f" %(self.current.heading,self.desire.heading,angleDiff)
        self.valCmd.setZ(turnSpeed)
        self.valCmd.push()
        return False

    def odom_callback(self,data):
        """
        update the state of the robot
        :type msg: Odom
        """
        self.current.x = data.pose.pose.position.x
        self.current.y = data.pose.pose.position.y
        quart = data.pose.pose.orientation 
        qq = [quart.x, quart.y, quart.z, quart.w] 
        self.current.heading = euler_from_quaternion(qq)[2]  
        self.current.seq = data.header.seq

    def planParser_callback(self,plan):
        # param plan: Path
        print "self seq: " , self.planSeq
        print ( "given seq :"+ str(plan.header.seq) )
        self.planSeq = plan.header.seq
        self.plan=[]
        for point in plan.poses:
            pos = self.PoseStruct(point.pose.position.x,point.pose.position.y,0)
            self.plan.append(pos)
            # print pos 
        print " recieved the new plan! "
        print "self seq: " , self.planSeq
        print("desire seq: " + str(self.desire.seq) )
        
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

