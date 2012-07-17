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
# test client for symbol_grounding_scan_base_pose_server
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
from srs_msgs.msg import SRSSpatialInfo


def symbol_grounding_scan_base_pose_client(parent_obj_geometry, furniture_geometry_list):

	rospy.wait_for_service('symbol_grounding_scan_base_pose')
	
	symbol_grounding_scan_base_pose = rospy.ServiceProxy('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose)
	
	try:
		resp = list()
		resp.append(symbol_grounding_scan_base_pose(parent_obj_geometry, furniture_geometry_list))
		return resp
	
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e



def getWorkspaceOnMap():
	print 'test get all workspace (furnitures basically here) from map'
	try:
		requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
		res = requestNewTask('ipa-kitchen-map', True)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



if __name__ == "__main__":

	workspace_info = getWorkspaceOnMap()	
	
	parent_obj_geometry = SRSSpatialInfo()
	
	parent_obj_geometry.pose.position.x = workspace_info.objectsInfo[2].pose.position.x
	parent_obj_geometry.pose.position.y = workspace_info.objectsInfo[2].pose.position.y
	parent_obj_geometry.pose.position.z = workspace_info.objectsInfo[2].pose.position.z
	parent_obj_geometry.pose.orientation.x = workspace_info.objectsInfo[2].pose.orientation.x
	parent_obj_geometry.pose.orientation.y = workspace_info.objectsInfo[2].pose.orientation.y
	parent_obj_geometry.pose.orientation.z = workspace_info.objectsInfo[2].pose.orientation.z
	parent_obj_geometry.pose.orientation.w = workspace_info.objectsInfo[2].pose.orientation.w
	parent_obj_geometry.l = workspace_info.objectsInfo[2].l
	parent_obj_geometry.w = workspace_info.objectsInfo[2].w
	parent_obj_geometry.h = workspace_info.objectsInfo[2].h
	rospy.loginfo(parent_obj_geometry.pose)

	#parent_obj_geometry.pose.position.x = 8.6
	#parent_obj_geometry.pose.position.y = 2.5
	#parent_obj_geometry.pose.orientation.x = 0 
	#parent_obj_geometry.pose.orientation.y = 0
	#parent_obj_geometry.pose.orientation.z = -0.999783754349
	#parent_obj_geometry.pose.orientation.w = 0.0207948293537
	#parent_obj_geometry.l = 1.5
	#parent_obj_geometry.w = 0.4

	furniture_geometry_list = list()
	furniture_geometry_list = workspace_info.objectsInfo


	print "Requesting scan base pose."
	
	result = symbol_grounding_scan_base_pose_client(parent_obj_geometry, furniture_geometry_list)
	
	print result
		


