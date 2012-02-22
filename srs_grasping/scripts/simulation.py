#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy

import grasping_functions
from srs_grasping.srv import *



class grasp_simulation():
	def __init__(self):

		rospy.loginfo("Waiting /get_model_mesh service...")
		rospy.wait_for_service('/get_model_mesh')
		rospy.loginfo("/get_model_mesh has been found!")

		rospy.loginfo("Waiting /get_grasp_configurations service...")
		rospy.wait_for_service('/get_grasp_configurations')
		rospy.loginfo("/get_grasp_configurations has been found!")

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self, object_id):	

		get_grasp_configurations = rospy.ServiceProxy('get_grasp_configurations', GetGraspConfigurations)
		req = GetGraspConfigurationsRequest(object_id=object_id)
		grasps = (get_grasp_configurations(req)).grasp_configuration

		grasping_functions.show_all_grasps(object_id, grasps);

		return 0

##########################################################################
if __name__ == "__main__":################################################
##########################################################################
    	rospy.init_node('grasp_simulation')
	s = grasp_simulation()
    	s.run(1)	#milk
