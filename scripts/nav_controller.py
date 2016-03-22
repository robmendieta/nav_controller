#!/usr/bin/env python
"""
This script is a simple python node which imports
source code of common_messages_examples.
"""
#-*- encoding: utf-8 -*-
__author__ = 'roberto_mendieta'

import rospy
import tf2_ros
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf2_msgs.msg import *
#from nav_class import *


class NavClass():
    def __init__(self):
	   #Listener
        self.parent_tfFrame = 'base_footprint'
        self.child_tfFrame = 'odom'
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfx = 0.0
        self.tfy = 0.0
        
        #Odometry
        self.odometry_msg = Odometry()
        self.originx = 0.0
        self.originy = 0.0
        
        #Pose 
        self.pose_msg = PoseStamped()
        
        #Publish speed
        self.speed_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.speed_msg = geometry_msgs.msg.Twist()
	
    def getOrigin(self,msg):
        self.odometry_msg = msg
        self.originx = self.odometry_msg.pose.pose.position.x
        self.originy = self.odometry_msg.pose.pose.position.y

        #TF Listener
        self.getTF()
         
    def getTF(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.parent_tfFrame,self.child_tfFrame, rospy.Time())
            self.tfx = trans.transform.translation.x
            self.tfy = trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()

        
    def runRobot(self):
        rospy.loginfo("Running the robot")
        
        #TF        
        rospy.loginfo("TF Listener parent: %s , child: %s ",self.parent_tfFrame,self.child_tfFrame)    
        rospy.loginfo("TF Listener position: x = %f , y = %f ",self.tfx,self.tfy)
        
        #Odom
        rospy.loginfo("nav_class origin frame_ID:" + self.odometry_msg.header.frame_id)	
        rospy.loginfo("nav_class child_frame_ID: %s" , self.odometry_msg.child_frame_id) 
        rospy.loginfo("position: x = %f , y = %f ",self.originx,self.originy)	
        
        #Pose
        rospy.loginfo("nav_class pose frame_ID:" + self.pose_msg.header.frame_id)	
        rospy.loginfo("position: x = %f , y = %f ",self.pose_msg.pose.position.x,self.pose_msg.pose.position.y)
        xgoal = self.pose_msg.pose.position.x
        ygoal = self.pose_msg.pose.position.y
        
        while xgoal-self.originx != 0 and ygoal-self.originy != 0:
            self.speed_msg.linear.x = xgoal-self.originx*0.1
            self.speed_msg.linear.y = ygoal-self.originy*0.1
            self.speed_pub.publish(self.speed_msg)
    
    def stopRobot(self):
        rospy.loginfo("Stopping the robot")
        self.speed_msg.linear.x = 0
        self.speed_msg.angular.z = 0
        self.speed_pub.publish(self.speed_msg)

			       
    def setOdom(self,msg):
        nav_class.getOrigin(msg)
	    
    def setPose(self,msg):
        rospy.loginfo("Subscribing to pose")
        self.pose_msg = msg
        
        

    def checkEventMsg(self,msg):
        # If the message is a e_start or e_stop, then call the function to move the robot
        if msg.data == 'e_start':
            self.runRobot()
            
        elif msg.data == 'e_stop':
            self.stopRobot()

if __name__ == '__main__':
    
    #Node init
    rospy.init_node('BaseController', anonymous=False)
    rospy.loginfo("nav_class node is running")
    
    nav_class = NavClass()

    #Subscribe to odom
    rospy.Subscriber("~/relative_pose", PoseStamped, nav_class.setPose)
    rospy.Subscriber("odom", Odometry, nav_class.setOdom)
    #rospy.Subscriber("tf",TFMessage,tf_cb)
    rospy.Subscriber("~/event_in", String, nav_class.checkEventMsg)
    rospy.spin()
    
