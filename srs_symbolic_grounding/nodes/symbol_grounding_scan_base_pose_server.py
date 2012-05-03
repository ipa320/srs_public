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
from srs_symbolic_grounding.srv import *
from srs_symbolic_grounding.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from nav_msgs.srv import *
#from srs_knowledge.msg import *
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import csv
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

def getMapClient():

	try:
		reqMap = rospy.ServiceProxy('static_map', GetMap)	
		res = reqMap()
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



def obstacleCheck(sbpl, fgl):
	obstacle_checked_scan_base_pose_list = list()
	wall_checked_scan_base_pose_list = list()
	scan_base_pose_list = sbpl
	furniture_geometry_list = fgl

	#obstacle check
	dist_to_obstacles = 0.5
	index_1 = 0
	while index_1 < len(scan_base_pose_list):
		index_2 = 0
		while index_2 < len(furniture_geometry_list):
			delta_x = math.sqrt((scan_base_pose_list[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (scan_base_pose_list[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(scan_base_pose_list[index_1].theta - furniture_geometry_list[index_2].pose.theta)
			delta_y = math.sqrt((scan_base_pose_list[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (scan_base_pose_list[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(scan_base_pose_list[index_1].theta - furniture_geometry_list[index_2].pose.theta)
			if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + dist_to_obstacles) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + dist_to_obstacles)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + dist_to_obstacles) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + dist_to_obstacles)):
				index_2 += 1
			else:
				break
		if index_2 == len(furniture_geometry_list):
			obstacle_checked_scan_base_pose_list.append(scan_base_pose_list[index_1])
		index_1 += 1
		#rospy.loginfo(obstacle_checked_scan_base_pose_list)

	if obstacle_checked_scan_base_pose_list:
			
		#wall check
		data = getMapClient()

		#rospy.loginfo(data.map.info)
		dist_to_walls = 0.5
		threshold = 10.0
		step_angle = 30.0

		index_3 = 0
		while index_3 < len(obstacle_checked_scan_base_pose_list):
			map_index_list = list()
			n = 0
			while n < int(360.0 / step_angle):
				wall_check_point_x = obstacle_checked_scan_base_pose_list[index_3].x + dist_to_walls * math.cos(n * step_angle / 180.0 * math.pi)
				wall_check_point_y = obstacle_checked_scan_base_pose_list[index_3].y + dist_to_walls * math.sin(n * step_angle / 180.0 * math.pi)
				
				#rospy.loginfo([wall_check_point_x, wall_check_point_y])

				map_index = int((wall_check_point_y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((wall_check_point_x - data.map.info.origin.position.x) / data.map.info.resolution)
				map_index_list.append(map_index)
				n += 1
				
			map_index = int((obstacle_checked_scan_base_pose_list[index_3].y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((obstacle_checked_scan_base_pose_list[index_3].x - data.map.info.origin.position.x) / data.map.info.resolution)
			map_index_list.append(map_index)
			#rospy.loginfo(map_index_list)

			index_4 = 0
			while index_4 < len(map_index_list):
				#rospy.loginfo(data.map.data[map_index_list[index_4]])
				if -1 < data.map.data[map_index_list[index_4]] < threshold:
					index_4 += 1
				else:
					break
			if index_4 == len(map_index_list):
				wall_checked_scan_base_pose_list.append(obstacle_checked_scan_base_pose_list[index_3])
			index_3 += 1
	
	return 	wall_checked_scan_base_pose_list			
	#rospy.loginfo(wall_checked_scan_base_pose_list)


def handle_symbol_grounding_scan_base_pose(req):

	'''
	#record the map for checking
	data = getMapClient()
	spamWriter = csv.writer(open('map_data.csv', 'wb'), delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	for n in range(80, 250):
		spamWriter.writerow(data.map.data[130 + 320 * n : 200 + 320 * n])
		n += 1
	'''

	'''
	#test the map
	data = getMapClient()
	#test points
	x = -3.2
	y = -0.58  
	map_index = int((y - data.map.info.origin.position.y) / data.map.info.resolution * data.map.info.width + (x - data.map.info.origin.position.x - 6.0) / data.map.info.resolution - 1)
	for n in range(-20, 100):	
		map_line = list()
		for m in range (-9, 9):
			map_line.append(data.map.data[map_index + n * int(data.map.info.width) + m])
			m += 1
		print map_line
		n += 1
	rospy.loginfo([data.map.info.origin.position.x, data.map.info.origin.position.y])
	rospy.loginfo([data.map.info.width, data.map.info.height])
	'''

	

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

	#rospy.loginfo(req.parent_obj_geometry)

	
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
	detection_angle = (30.0 / 180.0) * math.pi
	camera_distance = math.sqrt((robot_h - parent_obj_h) ** 2 + (rb_distance - 0.2) ** 2)
	detection_w = 2 * (camera_distance * math.tan(0.5 * detection_angle))	
	#rospy.loginfo(detection_w)






	

	scan_base_pose_1 = Pose2D()
	scan_base_pose_2 = Pose2D()
	scan_base_pose_3 = Pose2D()
	scan_base_pose_4 = Pose2D()
	

	scan_base_pose_list_1 = list()
	scan_base_pose_list_2 = list()
	scan_base_pose_list_3 = list()
	scan_base_pose_list_4 = list()


	

	if ((parent_obj_th >= 0) & (parent_obj_th <= (45.0 / 180.0 * math.pi))) | ((parent_obj_th >= (135.0 / 180.0 * math.pi)) & (parent_obj_th <= (225.0 / 180.0 * math.pi))) | ((parent_obj_th >= (315.0 / 180.0 * math.pi)) & (parent_obj_th < 360)):

		for num in range(int((parent_obj_l / detection_w) + 0.99)):
			scan_base_pose_1 = Pose2D()
			scan_base_pose_1.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_1.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 *  detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_1.theta = parent_obj_th + math.pi
			scan_base_pose_list_1.append(scan_base_pose_1)

		#rospy.loginfo(scan_base_pose_list_1)
		
				
		for num in range(int((parent_obj_l / detection_w) + 0.99)):
			scan_base_pose_2 = Pose2D()
			scan_base_pose_2.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_2.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_2.theta = parent_obj_th
			scan_base_pose_list_2.append(scan_base_pose_2)




		for num in range(int((parent_obj_w / detection_w) + 0.99)):

			scan_base_pose_3 = Pose2D()
			scan_base_pose_3.x = parent_obj_x + (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_3.y = parent_obj_y - (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_w - 0.5 *  detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_3.theta = parent_obj_th - 0.5 * math.pi
			scan_base_pose_list_3.append(scan_base_pose_3)

		


		for num in range(int((parent_obj_w / detection_w) + 0.99)):
			scan_base_pose_4 = Pose2D()
			scan_base_pose_4.x = parent_obj_x - (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_4.y = parent_obj_y + (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
			scan_base_pose_list_4.append(scan_base_pose_4)

		
	else:

		for num in range(int((parent_obj_w / detection_w) + 0.99)):

			scan_base_pose_1 = Pose2D()
			scan_base_pose_1.x = parent_obj_x - (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_1.y = parent_obj_y - (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_w - 0.5 *  detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_1.theta = parent_obj_th + math.pi
			scan_base_pose_list_1.append(scan_base_pose_1)


		
				
		for num in range(int((parent_obj_w / detection_w) + 0.99)):
			scan_base_pose_2 = Pose2D()
			scan_base_pose_2.x = parent_obj_x + (parent_obj_l * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_2.y = parent_obj_y + (parent_obj_l * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_w - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_2.theta = parent_obj_th
			scan_base_pose_list_2.append(scan_base_pose_2)


		

		for num in range(int((parent_obj_l / detection_w) + 0.99)):

			scan_base_pose_3 = Pose2D()
			scan_base_pose_3.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_3.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 *  detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_3.theta = parent_obj_th - 0.5 * math.pi
			scan_base_pose_list_3.append(scan_base_pose_3)

		

				
		for num in range(int((parent_obj_l / detection_w) + 0.99)):
			scan_base_pose_4 = Pose2D()
			scan_base_pose_4.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
			scan_base_pose_4.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
			scan_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
			scan_base_pose_list_4.append(scan_base_pose_4)
		
	#rospy.loginfo(scan_base_pose_list_1)
	#rospy.loginfo(scan_base_pose_list_2)
	#rospy.loginfo(scan_base_pose_list_3)
	#rospy.loginfo(scan_base_pose_list_4)
	
	#obstacle check
	obstacle_checked_scan_base_pose_list_1 = obstacleCheck(scan_base_pose_list_1, furniture_geometry_list)
	obstacle_checked_scan_base_pose_list_2 = obstacleCheck(scan_base_pose_list_2, furniture_geometry_list)
	obstacle_checked_scan_base_pose_list_3 = obstacleCheck(scan_base_pose_list_3, furniture_geometry_list)
	obstacle_checked_scan_base_pose_list_4 = obstacleCheck(scan_base_pose_list_4, furniture_geometry_list)



	#rospy.loginfo([obstacle_checked_scan_base_pose_list_1, obstacle_checked_scan_base_pose_list_2, obstacle_checked_scan_base_pose_list_3, obstacle_checked_scan_base_pose_list_4])
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

	

	return scan_base_pose_list



def symbol_grounding_scan_base_pose_server():
	rospy.init_node('symbol_grounding_scan_base_pose_server')
	s = rospy.Service('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose, handle_symbol_grounding_scan_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_scan_base_pose_server()


