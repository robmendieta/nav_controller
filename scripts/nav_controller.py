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
        #Pose 
        self.pose_msg = PoseStamped()
        
	   #Transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.originx = 0.0
        self.originy = 0.0
        self.parent = "apt_origin"
        self.child = "apt_robot"
                
        #Publish speed
        self.speed_pub = rospy.Publisher('cmd_vel_current', geometry_msgs.msg.Twist, queue_size=1)
        self.speed_msg = geometry_msgs.msg.Twist()
	
         
    def getTF(self):   
        #TF listener, can look for transforms
        #Should implement for flexibility and frame migration
        rate = rospy.Rate(10.0)
        try:
            trans = self.tfBuffer.lookup_transform(self.parent,self.child, rospy.Time())
            self.originx = trans.transform.translation.x
            self.originy = trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("EXCEPTION!!!!!!")
            rate.sleep()
   
    def runRobot(self):
        rospy.loginfo("Running the robot")
        
        rate = rospy.Rate(10.0)
        
        #Get origin
        #TF Listener
        self.getTF()    
        
        #Get distance
        distancex = self.pose_msg.pose.position.x - self.originx
        distancey = self.pose_msg.pose.position.y - self.originy
        rospy.loginfo("Distance x: %f" , distancex)
        rospy.loginfo("Distance y: %f" , distancey)
        
        while (distancex > 0.1 or distancex < -0.1) or (distancey > 0.1 or distancey < -0.1):
            #Get origin
            #TF Listener
            self.getTF()                
            
            #Odom (updated each loop)
            rospy.loginfo("APT_ORIGIN position: x = %f , y = %f ",self.originx , self.originy)	
        
            #Pose (from pose_msg)
            rospy.loginfo("nav_class pose frame_ID:" + self.pose_msg.header.frame_id)	
            rospy.loginfo("position: x = %f , y = %f ",self.pose_msg.pose.position.x , self.pose_msg.pose.position.y)
            
            distancex = self.pose_msg.pose.position.x-self.originx
            distancey = self.pose_msg.pose.position.y-self.originy
            rospy.loginfo("Distance x: %f" , distancex)
            rospy.loginfo("Distance y: %f" , distancey)
            
            #All other speeds to 0
            self.speed_msg.angular.x = 0.0
            self.speed_msg.angular.y = 0.0
            self.speed_msg.angular.z = 0.0
            self.speed_msg.linear.x = distancex*0.1
            self.speed_msg.linear.y = distancey*0.1
            self.speed_msg.linear.z = 0.0
            self.speed_pub.publish(self.speed_msg)
            rate.sleep()
            
        rospy.loginfo("Distance x: %f" , distancex)
        rospy.loginfo("Distance y: %f" , distancey)    
        self.stopRobot()
        
   
    def stopRobot(self):
        rospy.loginfo("Stopping the robot")
        self.speed_msg.linear.x = 0.0
        self.speed_msg.linear.y = 0.0
        self.speed_pub.publish(self.speed_msg)

			       
    def setPose(self,msg):
        self.pose_msg = msg
        rospy.loginfo("Subscribing to pose")

        
        

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
    rospy.Subscriber("~/event_in", String, nav_class.checkEventMsg)
    rospy.spin()
    
