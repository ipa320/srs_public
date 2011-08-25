#!/usr/bin/python

import time
import roslib
roslib.load_manifest('srs_grasping')
import rospy
import actionlib

import grasping_functions
from srs_grasping.msg import *



class grasp_action_server():

	def __init__(self):
		self.package_path = roslib.packages.get_pkg_dir('srs_grasping')
		self.ns_global_prefix = "/grasp_server"
		self.grasp_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, GetGraspsAction, self.execute_cb, True)
		self.grasp_action_server.start()
	
	def execute_cb(self, server_goal):	

		file_name = self.package_path+'/DB/'+server_goal.ObjectID+"_all_grasps.xml"
		server_result = GetGraspsActionResult().result

		try:
			GRASPS = grasping_functions.getGrasps(file_name, pose=server_goal.poseID, msg=True)
			g = GRASPS[0]	
			rospy.loginfo(str(len(g))+" grasping configuration for this object.")		


			if len(g)==0:
				print "----------------------------"
				print "Wrong poseID value. Options:"
				print "		 Z"
				print "		_Z"
				print "		 X"
				print "		_X"
				print "		 Y"
				print "		_Y"
				print "----------------------------"

				server_result.grasps = []
				server_result.response = False;	
				self.grasp_action_server.set_aborted(server_result)



			server_result.grasps = g
			server_result.response = True;
			self.grasp_action_server.set_succeeded(server_result)

		except:
			rospy.logerr("No grasping configurations for "+server_goal.ObjectID+" object.")	
			server_result.grasps = []
			server_result.response = False;		
			self.grasp_action_server.set_aborted(server_result)



## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('GetGraspsActionServer')
	SCRIPT = grasp_action_server()
	rospy.loginfo("/grasp_server is running")
	rospy.spin()
