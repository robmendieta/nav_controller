#!/usr/bin/env python
"""
This script is a simple python node which imports
source code of common_messages_examples.
"""
#-*- encoding: utf-8 -*-
__author__ = 'roberto_mendieta'

import rospy
import tf2_ros
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from tf2_msgs.msg import *
#from nav_class import *

#State machine
#Node init
#Logic

class BaseController():
    def __init__(self):
		#Listener
        self.parent_tfFrame = 'base_footprint'
        self.child_tfFrame = 'odom'
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        #Odometry
        self.odometry_msg = Odometry()
        
	
    def getOrigin(self,msg):
        self.odometry_msg = msg
        originx = self.odometry_msg.pose.pose.position.x
        originy = self.odometry_msg.pose.pose.position.y
        rospy.loginfo("nav_class origin frame_ID:" + self.odometry_msg.header.frame_id)	
        rospy.loginfo("nav_class child_frame_ID: %s" , self.odometry_msg.child_frame_id) 
        rospy.loginfo("position: x = %f , y = %f ",originx,originy)	
        
        self.getTF()
         
    def getTF(self):
        try:
            trans = self.tfBuffer.lookup_transform(self.parent_tfFrame,self.child_tfFrame, rospy.Time())
            tfx = trans.transform.translation.x
            tfy = trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
        rospy.loginfo("TF Listener parent: %s , child: %s ",self.parent_tfFrame,self.child_tfFrame)    
        rospy.loginfo("TF Listener position: x = %f , y = %f ",tfx,tfy)
        
			       
    def setOdom(msg):
	    nav_class.getOrigin(msg)
	
    #def tf_cb(msg):
	    #nav_class.getTF()
	    
	def setPose(msg):
        # If there is a message to subscribe, change the global PoseStamped-variable


    #If there is a message to subcribe from event_in, then start this function
    def checkEventMsg(msg):
        # If the message is a e_start or e_stop, then call the function to move the robot
        if msg.data == 'e_start' or msg.data == 'e_stop':
            if msg.data == 'e_start':
                #runTheRobot()
            else:
                #stopTheRobot()
                rospy.loginfo("Stop!") 

    if __name__ == '__main__':
        nav_class = BaseController()
    
        #Node init
        rospy.init_node('base_controller', anonymous=False)
        rospy.loginfo("nav_class node is running")
    
        #Subscribe to odom
        rospy.Subscriber("~/relative_pose", PoseStamped, setPose)
        rospy.Subscriber("odom", Odometry, setOdom)
        #rospy.Subscriber("tf",TFMessage,tf_cb)
        rospy.Subscriber("~/event_in", String, checkEventMsg)
        rospy.spin()
    
