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
#   Author: Manuel Rodriguez, email:mrodriguez@robotnik.es
# \author
#   Supervised by: Manuel Rodriguez, email:mrodriguez@robotnik.es
#
# \date Date of creation: March 2012
#
# \brief
#   Implements the simulation for the precomputed grasps. 
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
    	s.run(9)	#milk
