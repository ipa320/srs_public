#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_grasping')
import rospy
import time

import actionlib
from srs_grasping.msg import *
from srs_grasping import *
from std_msgs import *

def grasp_action_client():

	x = time.time()
	client = actionlib.SimpleActionClient('/grasp_server', GraspAction)
	client.wait_for_server()

	goal = GraspGoal(object_id=0)
	client.send_goal(goal)
	client.wait_for_result()
	y = time.time()
	print y-x
	return client.get_result() 

if __name__ == '__main__':
	try:
		rospy.init_node('get_grasps_action_client')
		result = grasp_action_client()
		print "-----------------"
		print result
		print "-----------------"
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
