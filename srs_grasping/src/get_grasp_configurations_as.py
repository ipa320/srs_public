#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import time
import rospy
import actionlib

import grasping_functions
from srs_grasping.msg import *

class get_grasp_configurations():

	def __init__(self):

		rospy.loginfo("Waiting /get_model_grasp service...")
		rospy.wait_for_service('/get_model_grasp')
		rospy.loginfo("/get_model_grasp is ready.")

		self.ns_global_prefix = "/get_grasp_configurations"
		self.get_grasp_configurations = actionlib.SimpleActionServer(self.ns_global_prefix, GraspCAction, self.execute_cb, True)
		rospy.loginfo("/get_grasp_configurations is ready.")

		self.get_grasp_configurations.start()

	
	def execute_cb(self, server_goal):
		x = time.time()
		rospy.loginfo("/get_grasp_configurations_server has been called...");
		server_result = GraspCActionResult().result

		fail = False;

		resp = grasping_functions.read_grasps_from_DB(server_goal.object_id);
		if resp == -1:
			resp = grasping_functions.generator(server_goal.object_id);
			if resp != -1:
				resp = grasping_functions.read_grasps_from_DB(server_goal.object_id);
				server_result.grasp_configuration = resp.grasp_configuration;
			else:
				fail = True;
		else:
			server_result.grasp_configuration = resp.grasp_configuration;

		if fail:
			self.get_grasp_configurations.set_aborted(server_result)
		else:
			self.get_grasp_configurations.set_succeeded(server_result)

		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('get_grasp_configurations')
	SCRIPT = get_grasp_configurations()
	rospy.spin()
