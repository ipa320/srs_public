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
# ROS stack name: srs
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
# Grounding scan object surface commands
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

from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
'''
def getWorkspaceOnMap():
	print 'test get all workspace (furnitures basically here) from map'
	try:
		requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
		res = requestNewTask('ipa-kitchen-map', True)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
'''
def handle_symbol_grounding_scan_base_pose(req):

	'''	
	#get furniture information from knowledge base
	workspace_info = getWorkspaceOnMap()	
	furniture_geometry_list = list()
	furniture_geometry_list = workspace_info.objectsInfo
	
	
	#transfrom list
	index = 0
	furniture_geometry_list = list()
	while index < len(furniture_geometry_list):
		furniture_geometry = FurnitureGeometry()
		furniture_geometry.pose.x = furniture_geometry_list[index].pose.position.x
		furniture_geometry.pose.y = furniture_geometry_list[index].pose.position.y
		furniture_pose_rpy = tf.transformations.euler_from_quaternion([furniture_geometry_list[index].pose.orientation.x, furniture_geometry_list[index].pose.orientation.y, furniture_geometry_list[index].pose.orientation.z, furniture_geometry_list[index].pose.orientation.w])		
		furniture_geometry.pose.theta = furniture_pose_rpy[2]
		furniture_geometry.l = furniture_geometry_list[index].l
		furniture_geometry.w = furniture_geometry_list[index].w
		furniture_geometry.h = furniture_geometry_list[index].h
		furniture_geometry_list.append(furniture_geometry)
		index += 1


	
	'''	
	parent_obj_x = req.parent_obj_geometry.pose.position.x
	parent_obj_y = req.parent_obj_geometry.pose.position.y
	parent_obj_rpy = tf.transformations.euler_from_quaternion([req.parent_obj_geometry.pose.orientation.x, req.parent_obj_geometry.pose.orientation.y, req.parent_obj_geometry.pose.orientation.z, req.parent_obj_geometry.pose.orientation.w])
	parent_obj_th = parent_obj_rpy[2]
	parent_obj_l = req.parent_obj_geometry.l
	parent_obj_w = req.parent_obj_geometry.w
	parent_obj_h = req.parent_obj_geometry.h

	rospy.loginfo(req.parent_obj_geometry)

	
	#transfrom list
	index = 0
	furniture_geometry_list = list()
	while index < len(req.furniture_geometry_list):
		furniture_geometry = FurnitureGeometry()
		furniture_geometry.pose.x = req.furniture_geometry_list[index].pose.position.x
		furniture_geometry.pose.y = req.furniture_geometry_list[index].pose.position.y
		furniture_pose_rpy = tf.transformations.euler_from_quaternion([req.furniture_geometry_list[index].pose.orientation.x, req.furniture_geometry_list[index].pose.orientation.y, req.furniture_geometry_list[index].pose.orientation.z, req.furniture_geometry_list[index].pose.orientation.w])		
		furniture_geometry.pose.theta = furniture_pose_rpy[2]
		furniture_geometry.l = req.furniture_geometry_list[index].l
		furniture_geometry.w = req.furniture_geometry_list[index].w
		furniture_geometry.h = req.furniture_geometry_list[index].h
		furniture_geometry_list.append(furniture_geometry)
		index += 1
	
	

	#get detection width
	rb_distance = 0.7
	robot_h = 1.4
	detection_angle = (45.0 / 180.0) * math.pi
	camera_distance = math.sqrt((robot_h - parent_obj_h) ** 2 + (rb_distance - 0.2) ** 2)
	detection_w = 2 * (camera_distance * math.tan(0.5 * detection_angle))	






	

	scan_base_pose_1 = Pose2D()
	scan_base_pose_2 = Pose2D()
	scan_base_pose_3 = Pose2D()
	scan_base_pose_4 = Pose2D()
	

	scan_base_pose_list_1 = list()
	scan_base_pose_list_2 = list()
	scan_base_pose_list_3 = list()
	scan_base_pose_list_4 = list()

	wall_checked_scan_base_pose_list_1 = list()
	wall_checked_scan_base_pose_list_2 = list()
	wall_checked_scan_base_pose_list_3 = list()
	wall_checked_scan_base_pose_list_4 = list()

	obstacle_checked_scan_base_pose_list_1 = list()
	obstacle_checked_scan_base_pose_list_2 = list()
	obstacle_checked_scan_base_pose_list_3 = list()
	obstacle_checked_scan_base_pose_list_4 = list()
	

	if ((parent_obj_th >= 0) & (parent_obj_th <= (45.0 / 180.0 * math.pi))) | ((parent_obj_th >= (135.0 / 180.0 * math.pi)) & (parent_obj_th <= (225.0 / 180.0 * math.pi))) | ((parent_obj_th >= (315.0 / 180.0 * math.pi)) & (parent_obj_th < 360)):

		for num in range(int((parent_obj_l / detection_w) + 0.99)):

			scan_base_pose_1 = Pose2D()
			scan_base_pose_1.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_1.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 *  detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_1.theta = parent_obj_th + math.pi
			scan_base_pose_list_1.append(scan_base_pose_1)


		#obstacle check 1


		index = 0
		while index < len(scan_base_pose_list_1):
			if ((-2.7 <= scan_base_pose_list_1[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_1[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_1[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_1[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_1.append(scan_base_pose_list_1[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_1):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_1[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_1[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_1[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_1[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_1.append(wall_checked_scan_base_pose_list_1[index_1])
				index_1 += 1
	
				
		for num in range(int((parent_obj_l / detection_w) + 0.99)):
			scan_base_pose_2 = Pose2D()
			scan_base_pose_2.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_2.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_2.theta = parent_obj_th
			scan_base_pose_list_2.append(scan_base_pose_2)


		#obstacle check 2


		index = 0
		while index < len(scan_base_pose_list_2):
			if ((-2.7 <= scan_base_pose_list_2[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_2[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_2[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_2[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_2.append(scan_base_pose_list_2[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_2):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_2[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_2[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_2[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_2[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_2[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_2.append(wall_checked_scan_base_pose_list_2[index_1])
				index_1 += 1

		for num in range(int((parent_obj_w / detection_w) + 0.99)):

			scan_base_pose_3 = Pose2D()
			scan_base_pose_3.x = parent_obj_x + (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_3.y = parent_obj_y - (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_w - 0.5 *  detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_3.theta = parent_obj_th - 0.5 * math.pi
			scan_base_pose_list_3.append(scan_base_pose_3)

		#obstacle check 3



		index = 0
		while index < len(scan_base_pose_list_3):
			if ((-2.7 <= scan_base_pose_list_3[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_3[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_3[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_3[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_3.append(scan_base_pose_list_3[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_3):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_3[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_3[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_3[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_3[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_3[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_3[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_3.append(wall_checked_scan_base_pose_list_3[index_1])
				index_1 += 1
				
		for num in range(int((parent_obj_w / detection_w) + 0.99)):
			scan_base_pose_4 = Pose2D()
			scan_base_pose_4.x = parent_obj_x - (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_4.y = parent_obj_y + (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
			scan_base_pose_list_4.append(scan_base_pose_4)

		#obstacle check 4



		index = 0
		while index < len(scan_base_pose_list_4):
			if ((-2.7 <= scan_base_pose_list_4[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_4[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_4[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_4[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_4.append(scan_base_pose_list_4[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_4):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_4[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_4[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_4[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_4[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_4[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_4.append(wall_checked_scan_base_pose_list_4[index_1])
				index_1 += 1

	else:

		for num in range(int((parent_obj_w / detection_w) + 0.99)):

			scan_base_pose_1 = Pose2D()
			scan_base_pose_1.x = parent_obj_x - (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_1.y = parent_obj_y - (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_w - 0.5 *  detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_1.theta = parent_obj_th + math.pi
			scan_base_pose_list_1.append(scan_base_pose_1)


		#obstacle check 1


		index = 0
		while index < len(scan_base_pose_list_1):
			if ((-2.7 <= scan_base_pose_list_1[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_1[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_1[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_1[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_1.append(scan_base_pose_list_1[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_1):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_1[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_1[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_1[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_1[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_1.append(wall_checked_scan_base_pose_list_1[index_1])
				index_1 += 1
	
				
		for num in range(int((parent_obj_w / detection_w) + 0.99)):
			scan_base_pose_2 = Pose2D()
			scan_base_pose_2.x = parent_obj_x + (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_2.y = parent_obj_y + (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_2.theta = parent_obj_th
			scan_base_pose_list_2.append(scan_base_pose_2)


		#obstacle check 2


		index = 0
		while index < len(scan_base_pose_list_2):
			if ((-2.7 <= scan_base_pose_list_2[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_2[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_2[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_2[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_2.append(scan_base_pose_list_2[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_2):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_2[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_2[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_2[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_2[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_2[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_2.append(wall_checked_scan_base_pose_list_2[index_1])
				index_1 += 1

		for num in range(int((parent_obj_l / detection_w) + 0.99)):

			scan_base_pose_3 = Pose2D()
			scan_base_pose_3.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_3.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 *  detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_3.theta = parent_obj_th - 0.5 * math.pi
			scan_base_pose_list_3.append(scan_base_pose_3)

		#obstacle check 3



		index = 0
		while index < len(scan_base_pose_list_3):
			if ((-2.7 <= scan_base_pose_list_3[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_3[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_3[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_3[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_3.append(scan_base_pose_list_3[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_3):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_3[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_3[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_3[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_3[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_3[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_3[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_3.append(wall_checked_scan_base_pose_list_3[index_1])
				index_1 += 1
				
		for num in range(int((parent_obj_l / detection_w) + 0.99)):
			scan_base_pose_4 = Pose2D()
			scan_base_pose_4.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_4.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
			scan_base_pose_list_4.append(scan_base_pose_4)

		#obstacle check 4



		index = 0
		while index < len(scan_base_pose_list_4):
			if ((-2.7 <= scan_base_pose_list_4[index].x <= 1.6) and (-1.7 <= scan_base_pose_list_4[index].y <= 1.2)) or ((1.6 <= scan_base_pose_list_4[index].x <= 3.2) and (-1.7 <= scan_base_pose_list_4[index].y <= 0.7)):
				wall_checked_scan_base_pose_list_4.append(scan_base_pose_list_4[index])
			index += 1
		
	
		else:
			index_1 = 0
			while index_1 < len(wall_checked_scan_base_pose_list_4):
				index_2 = 0
				while index_2 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_scan_base_pose_list_4[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_4[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list_1[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					delta_y = math.sqrt((wall_checked_scan_base_pose_list_4[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list_4[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list_4[index_1].theta - furniture_geometry_list[index_2].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
						index_2 += 1
					else:
						index_1 += 1
						break
				obstacle_checked_scan_base_pose_list_4.append(wall_checked_scan_base_pose_list_4[index_1])
				index_1 += 1

	rospy.loginfo([obstacle_checked_scan_base_pose_list_1, obstacle_checked_scan_base_pose_list_2, obstacle_checked_scan_base_pose_list_3, obstacle_checked_scan_base_pose_list_4])
	max_len = max(len(obstacle_checked_scan_base_pose_list_1), len(obstacle_checked_scan_base_pose_list_2), len(obstacle_checked_scan_base_pose_list_3), len(obstacle_checked_scan_base_pose_list_4))


	if len(obstacle_checked_scan_base_pose_list_1) == max_len:
		scan_base_pose_list = [obstacle_checked_scan_base_pose_list_1]
	elif len(obstacle_checked_scan_base_pose_list_2) == max_len:
		scan_base_pose_list = [obstacle_checked_scan_base_pose_list_2]
	elif len(obstacle_checked_scan_base_pose_list_3) == max_len:
		scan_base_pose_list = [obstacle_checked_scan_base_pose_list_3]
	else:
		scan_base_pose_list = [obstacle_checked_scan_base_pose_list_4]
	

	if not scan_base_pose_list:
		print "no valid scan pose."

	else:

		return scan_base_pose_list



def symbol_grounding_scan_base_pose_server():
	rospy.init_node('symbol_grounding_scan_base_pose_server')
	s = rospy.Service('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose, handle_symbol_grounding_scan_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_scan_base_pose_server()


