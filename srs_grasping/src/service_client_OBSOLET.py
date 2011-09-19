#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_grasping')
import rospy

import actionlib
from srs_grasping.msg import *
from srs_grasping.srv import *


def grasp_service_client():

	rospy.wait_for_service('/grasp_service/get_grasps')
	gg = rospy.ServiceProxy('/grasp_service/get_grasps', GetGrasps)  
	try:
		resp = gg("milk_box","Z")
	except rospy.ServiceException, e:
		print "Service did not process request: %s"%str(e)

	return resp



if __name__ == '__main__':
	try:
		rospy.init_node('get_grasp_service_client')
		result = grasp_service_client()
		print "-----------------"
		print result
		print "-----------------"
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
