#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# University of Bedfordshire \n\n
#
#################################################################
#
# \note
# Project name: care-o-bot
# \note
# ROS stack name: srs_public
# \note
# ROS package name: srs_symbolic_grounding
#
# \author
# Author: Beisheng Liu, email:beisheng.liu@beds.ac.uk
# \author
# Supervised by: Dayou Li, email:dayou.li@beds.ac.uk
#
# \date Date of creation: Mar 2012
#
# \brief
# test client for symbol_grounding_grasp_base_pose_experimental_server
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the University of Bedfordshire nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
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
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import *
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import tf
from srs_msgs.msg import SRSSpatialInfo
import math

def symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, parent_obj_geometry, furniture_geometry_list):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose_experimental')
	symbol_grounding_grasp_base_pose_experimental = rospy.ServiceProxy('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental)

	try:
		resp1 = symbol_grounding_grasp_base_pose_experimental(target_obj_pose, parent_obj_geometry, furniture_geometry_list)
		return resp1
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e


'''		
def getWorkspaceOnMap():
	#print 'test get all workspace (furnitures basically here) from map'
	try:
		requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
		resp2 = requestNewTask('ipa-kitchen-map', True)
		return resp2
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
'''		



if __name__ == "__main__":



	target_obj_pose = Pose()

	target_obj_pose.position.x = 0.64
	target_obj_pose.position.y = 0.9
	target_obj_pose.position.z = 0.8
	target_obj_pose.orientation.x = 0
	target_obj_pose.orientation.y = 0
	target_obj_pose.orientation.z = 0
	target_obj_pose.orientation.w = 1


	#workspace_info = getWorkspaceOnMap()	
	
	parent_obj_geometry = SRSSpatialInfo()
	
	'''
	parent_obj_geometry.pose.position.x = workspace_info.objectsInfo[0].pose.position.x
	parent_obj_geometry.pose.position.y = workspace_info.objectsInfo[0].pose.position.y
	parent_obj_geometry.pose.position.z = workspace_info.objectsInfo[0].pose.position.z
	parent_obj_geometry.pose.orientation.x = workspace_info.objectsInfo[0].pose.orientation.x
	parent_obj_geometry.pose.orientation.y = workspace_info.objectsInfo[0].pose.orientation.y
	parent_obj_geometry.pose.orientation.z = workspace_info.objectsInfo[0].pose.orientation.z
	parent_obj_geometry.pose.orientation.w = workspace_info.objectsInfo[0].pose.orientation.w
	parent_obj_geometry.l = workspace_info.objectsInfo[0].l
	parent_obj_geometry.w = workspace_info.objectsInfo[0].w
	parent_obj_geometry.h = workspace_info.objectsInfo[0].h
	'''

	parent_obj_geometry.pose.position.x = -3.3
	parent_obj_geometry.pose.position.y = 0.5
	parent_obj_geometry.pose.position.z = 1.0
	parent_obj_geometry.pose.orientation.x = 0
	parent_obj_geometry.pose.orientation.y = 0
	parent_obj_geometry.pose.orientation.z = 0
	parent_obj_geometry.pose.orientation.w = 1.0

	parent_obj_geometry.l = 2.0
	parent_obj_geometry.w = 1.0
	parent_obj_geometry.h = 0.85




	furniture_geometry_list = list()
	#furniture_geometry_list = workspace_info.objectsInfo
	#rospy.loginfo(workspace_info.objectsInfo[6])


	print "Requesting reachability and grasp base pose."
	result = symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, parent_obj_geometry, furniture_geometry_list)
	print result
	#print math.atan(0.26 / 0.7)
		

	

