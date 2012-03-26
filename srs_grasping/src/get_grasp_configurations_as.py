#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2012 \n
#   Robotnik Automation SLL \n\n
#
#################################################################
#
# \note
#   Project name: srs
# \note
#   ROS stack name: srs
# \note
#   ROS package name: srs_grasping
#
# \author
#   Author: Manuel Rodríguez, email:mrodriguez@robotnik.es
# \author
#   Supervised by: Manuel Rodríguez, email:mrodriguez@robotnik.es
#
# \date Date of creation: March 2012
#
# \brief
#   Implements an action server that returns all the grasping configurations for a given object_id.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Robotnik Automation SLL nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

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
