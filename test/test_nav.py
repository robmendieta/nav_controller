#!/usr/bin/env python
import sys
import unittest, time
import rospy, rostest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import String

class test_nav(unittest.TestCase):
	def __init__(self,*args):
		super(test_pub,self).__init__(*args)
		self.success = False
	def callback(self,data):
		if data.data == 'e_done':
			self.success = True
		
	def test_nav_node(self):
		rospy.init_node('test_nav')
		relative_pose_pub = rospy.Publisher("~relative_pose", PoseStamped, queue_size=100)
    		relative_pose_msg = PoseStamped()
    		relative_pose_msg.pose.position.x = 2.0
		relative_pose_msg.pose.position.y = 2.0
    		loop_rate = rospy.Rate(10)
    		eventout_pub.publish(relative_pose_msg)
    		loop_rate.sleep()
		rospy.Subscriber("~/event_out", String, self.callback)
		time_t=time.time()+10
		while(not rospy.is_shutdown() and not self.success and time.time() < time_t):
			time.sleep(0.1)
		self.assert_(self.success)
		

if __name__ == '__main__':
	rostest.rosrun('nav_controller','test_nav',test_nav)
