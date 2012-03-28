#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import time
import rospy

import grasping_functions
from srs_grasping.srv import *


class get_grasp_configurations():

	def __init__(self):
		rospy.loginfo("Waiting /get_model_grasp service...");
		rospy.wait_for_service('/get_model_grasp');
		rospy.loginfo("/get_model_grasp is ready.");
		rospy.loginfo("/get_grasp_configurations service is ready.");
		print "-------------------------------------------------------------------------";


	def get_grasp_configurations(self, server_goal):
		x = time.time();
		rospy.loginfo("/get_grasp_configurations service has been called...");


		server_result = GetGraspConfigurationsResponse();

		resp = grasping_functions.read_grasps_from_DB(server_goal.object_id);
		if resp == -1:
			resp = grasping_functions.generator(server_goal.object_id);
			if resp != -1:
				resp = grasping_functions.read_grasps_from_DB(server_goal.object_id);
				server_result.grasp_configuration = resp.grasp_configuration;
		else:
			server_result.grasp_configuration = resp.grasp_configuration;


		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";
		return server_result;


	def get_grasp_configurations_server(self):
		s = rospy.Service('/get_grasp_configurations', GetGraspConfigurations, self.get_grasp_configurations);


if __name__ == '__main__':
	rospy.init_node('get_grasp_configurations')
	SCRIPT = get_grasp_configurations()
	SCRIPT.get_grasp_configurations_server();
	rospy.spin()
