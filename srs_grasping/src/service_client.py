#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_grasping')
import rospy

import actionlib
from srs_grasping.msg import *
from srs_grasping.srv import *


def grasp_service_client():

	rospy.wait_for_service('/grasp_service/GetGrasps')
	gg = rospy.ServiceProxy('/grasp_service/GetGrasps', GetGrasps)  
	try:
		resp = gg("milk_box","Z")
	except rospy.ServiceException, e:
		print "Service did not process request: %s"%str(e)

	return resp



if __name__ == '__main__':
	try:
		rospy.init_node('GetGraspsServiceClient')
		result = grasp_service_client()
		print "-----------------"
		print result
		print "-----------------"
	except rospy.ROSInterruptException:
		print "program interrupted before completion"
