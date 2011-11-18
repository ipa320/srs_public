#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_grasping')
import rospy
import time
import actionlib

from srs_grasping.msg import *


def grasp_action_client():

	x = time.time()
	client = actionlib.SimpleActionClient('/grasp_server', GraspAction)
	client.wait_for_server()

	goal = GraspGoal(object_id=1, pose_id="X")	#Milk, 5 X-axis configurations
	client.send_goal(goal)
	client.wait_for_result()
	y = time.time()

	print y-x
	return client.get_result() 

if __name__ == '__main__':
	rospy.init_node('get_grasps_action_client')
	try:
		result = grasp_action_client()
		print "-----------------"
		print result
		print "-----------------"
	except rospy.ROSInterruptException:
		print "Program interrupted before completion"
