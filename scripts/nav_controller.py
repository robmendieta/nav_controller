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
        #Create Pose transform
        self.pose_transform = geometry_msgs.msg.TransformStamped()
        #Pose transform broadcaster
        self.pub_tf = tf2_ros.TransformBroadcaster()       

        #Odometry msg init
        self.odometry_msg = Odometry()
        self.originx = 0.0
        self.originy = 0.0
        
	   #Transform listener
        self.parent_tfFrame = 'odom'
        self.child_tfFrame = 'base_footprint'
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfx = 0.0
        self.tfy = 0.0
                
        #Publish speed
        self.speed_pub = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
        self.speed_msg = geometry_msgs.msg.Twist()
	
    def getOrigin(self,msg):
        self.odometry_msg = msg
        self.originx = self.odometry_msg.pose.pose.position.x
        self.originy = self.odometry_msg.pose.pose.position.y
         
    def getTF(self):
        #Fill pose_transform with data from pose_msg
        self.pose_transform.transform.translation.x = self.pose_msg.pose.position.x
        self.pose_transform.transform.translation.y = self.pose_msg.pose.position.y
        self.pose_transform.transform.translation.z = self.pose_msg.pose.position.z
        
        self.pose_transform.transform.rotation.x = self.pose_msg.pose.orientation.x
        self.pose_transform.transform.rotation.y = self.pose_msg.pose.orientation.y
        self.pose_transform.transform.rotation.z = self.pose_msg.pose.orientation.z
        self.pose_transform.transform.rotation.w = self.pose_msg.pose.orientation.w
        
        self.pose_transform.header.frame_id = "odom"
        self.pose_transform.header.stamp = rospy.Time.now()
        self.pose_transform.child_frame_id = "pose_frame"
        
        #Send transform
        self.pub_tf.sendTransform(self.pose_transform)
        
        #TF listener, can llok for transforms
        #Should implement for flexibility and frame migration
        rate = rospy.Rate(10.0)
        try:
            self.trans = self.tfBuffer.lookup_transform(self.parent_tfFrame,self.child_tfFrame, rospy.Time())
            self.tfx = self.trans.transform.translation.x
            self.tfy = self.trans.transform.translation.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("EXCEPTION!!!!!!")
            rate.sleep()
   
    def runRobot(self):
        rospy.loginfo("Running the robot")
        
        rate = rospy.Rate(10.0)
        
        #Get distance
        distancex = self.pose_msg.pose.position.x - self.originx
        distancey = self.pose_msg.pose.position.y - self.originy
        rospy.loginfo("Distance x: %f" , distancex)
        rospy.loginfo("Distance y: %f" , distancey)
        
        while (distancex > 0.01 or distancex < -0.01) and (distancey > 0.01 or distancey < -0.01):
            #TF listener values (only updated at start)
            rospy.loginfo("TF Listener parent:" + self.trans.header.frame_id + " child: " + self.trans.child_frame_id)    
            rospy.loginfo("TF Listener position: x = %f , y = %f " , self.tfx , self.tfy)
        
            #Odom (updated each loop)
            rospy.loginfo("ODOM position: x = %f , y = %f ",self.originx , self.originy)	
        
            #Pose (from pose_msg)
            rospy.loginfo("nav_class pose frame_ID:" + self.pose_msg.header.frame_id)	
            rospy.loginfo("position: x = %f , y = %f ",self.pose_msg.pose.position.x , self.pose_msg.pose.position.y)
            rate.sleep()
            
            distancex = self.pose_msg.pose.position.x-self.originx
            distancey = self.pose_msg.pose.position.y-self.originy
            rospy.loginfo("Distance x: %f" , distancex)
            rospy.loginfo("Distance y: %f" , distancey)
            
            #All other speeds to 0
            self.speed_msg.angular.x = 0.0
            self.speed_msg.angular.y = 0.0
            self.speed_msg.angular.z = 0.0
            self.speed_msg.linear.x = distancex
            self.speed_msg.linear.y = distancey
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

			       
    def setOdom(self,msg):
        self.getOrigin(msg)
	    
    def setPose(self,msg):
        self.pose_msg = msg
        rospy.loginfo("Subscribing to pose")
        #TF Listener
        self.getTF()    
        
        

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
    rospy.Subscriber("~/event_in", String, nav_class.checkEventMsg)
    rospy.spin()
    
