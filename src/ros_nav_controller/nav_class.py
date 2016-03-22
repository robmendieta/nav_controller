#!/usr/bin/env python
"""
This script is a simple python node which imports
source code of common_messages_examples.
"""
#-*- encoding: utf-8 -*-
__author__ = 'roberto_mendieta'

import rospy
from nav_msgs.msg import Odometry


class NavClass():
	def __init__(self):

    def getGoal(self,msg):
		odometry_msg = Odometry()
	    rospy.loginfo("nav_class node is looking for goal" + odometry_msg.child_frame_id)	
		
    



    
