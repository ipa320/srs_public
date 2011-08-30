#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_grasping')
import rospy

import actionlib
from srs_grasping.msg import *
from srs_grasping import *

def grasp_action_client():

	client = actionlib.SimpleActionClient('/grasp_server', GetGraspsAction)
	client.wait_for_server()
	goal = GetGraspsGoal(ObjectID="milk_box", poseID="Z")
	client.send_goal(goal)
	client.wait_for_result()
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
