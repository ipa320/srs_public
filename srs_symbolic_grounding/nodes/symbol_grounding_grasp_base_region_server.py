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
# \date Date of creation: April 2012
#
# \brief
# Grounding grasp object commands
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
#from srs_symbolic_grounding.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from nav_msgs.msg import *
from nav_msgs.srv import *
#from srs_knowledge.msg import *
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
#import csv



'''
#get furniture list from the knowledge base (part 1)
def getWorkspaceOnMap():
	print 'test get all workspace (furnitures basically here) from map'
	try:
		requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
		res = requestNewTask('ipa-kitchen-map', True)
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
'''

#get the occupancy grid map from the navigation services
def getMapClient():

	try:
		reqMap = rospy.ServiceProxy('static_map', GetMap)	
		res = reqMap()
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

#this function is used to calculate a list of canidate poses around the target obj. thre number of candidate poses is 360/step_angle
def getRobotBasePoseList(angle, dist, rbp, obj_x, obj_y):
	grasp_base_pose_list = list()
	step_angle = angle
	dist_to_obj = dist
	rb_pose = rbp
	target_obj_x = obj_x
	target_obj_y = obj_y
	th = math.atan((rb_pose.y - target_obj_y) / (rb_pose.x - target_obj_x))
	if rb_pose.x < target_obj_x and rb_pose.y > target_obj_y:
		th = math.pi + th
	if rb_pose.x < target_obj_x and rb_pose.y < target_obj_y:
		th = -math.pi + th
	#rospy.loginfo(th)
	for n in range (0, int(2 * math.pi / step_angle)):
		grasp_base_pose = Pose2D()
		grasp_base_pose.x = target_obj_x + dist_to_obj * math.cos(th) - 0.1 * math.sin(th) - 2 * math.sqrt(dist_to_obj ** 2 + 0.1 ** 2) * math.sin(0.5 * n * step_angle) * math.sin(0.5 * n * step_angle + math.atan(0.1 / dist_to_obj) + th)
		grasp_base_pose.y = target_obj_y + dist_to_obj * math.sin(th) + 0.1 * math.cos(th) + 2 * math.sqrt(dist_to_obj ** 2 + 0.1 ** 2) * math.sin(0.5 * n * step_angle) * math.cos(0.5 * n * step_angle + math.atan(0.1 / dist_to_obj) + th)
		grasp_base_pose.theta = th + n * step_angle
		#rospy.loginfo([th, n * step_angle, math.atan(0.1 / dist_to_obj)])
		if grasp_base_pose.theta > math.pi:
			grasp_base_pose.theta -= 2 * math.pi
		grasp_base_pose_list.append(grasp_base_pose)
	#rospy.loginfo(grasp_base_pose_list)
	return grasp_base_pose_list


	
#function for checking if a pose is obstacle free.
def obstacleCheck(gbpl, po_x, po_y, po_th, po_w, po_l, fgl):
	parent_obj_checked_grasp_base_pose_list = list()
	obstacle_checked_grasp_base_pose_list = list()
	wall_checked_grasp_base_pose_list = list()
	grasp_base_pose_list = gbpl
	parent_obj_x = po_x
	parent_obj_y = po_y
	parent_obj_th = po_th
	parent_obj_w = po_w
	parent_obj_l = po_l
	furniture_geometry_list = fgl
	
	#to check if a pose is too close to the edge of the parent obj. the valid poses will be stored in the parent_obj_checked_grasp_base_pose_list.
	dist_to_table = 0.45 #minimum distance to the edge of the parent obj
	index_1 = 0
	while index_1 < len(grasp_base_pose_list):
		th = math.atan((grasp_base_pose_list[index_1].y - parent_obj_y) / (grasp_base_pose_list[index_1].x - parent_obj_x))
		if grasp_base_pose_list[index_1].x < parent_obj_x and grasp_base_pose_list[index_1].y > parent_obj_y:
			th = math.pi + th
		if grasp_base_pose_list[index_1].x < parent_obj_x and grasp_base_pose_list[index_1].y < parent_obj_y:
			th = -math.pi + th
		delta_x = math.sqrt((grasp_base_pose_list[index_1].x - parent_obj_x) ** 2 + (grasp_base_pose_list[index_1].y - parent_obj_y) ** 2) * math.cos(th - parent_obj_th)
		delta_y = math.sqrt((grasp_base_pose_list[index_1].x - parent_obj_x) ** 2 + (grasp_base_pose_list[index_1].y - parent_obj_y) ** 2) * math.sin(th - parent_obj_th)
		#rospy.loginfo([delta_x, delta_y])
		if (delta_x <= -(parent_obj_w / 2.0 + dist_to_table) or delta_x >= (parent_obj_w / 2.0 + dist_to_table)) or (delta_y <= -(parent_obj_l / 2.0 + dist_to_table) or delta_y >= (parent_obj_l / 2.0 + dist_to_table)):
			parent_obj_checked_grasp_base_pose_list.append(grasp_base_pose_list[index_1])
		index_1 += 1
	#rospy.loginfo(parent_obj_checked_grasp_base_pose_list)
	
		
	if parent_obj_checked_grasp_base_pose_list: #if there is a parent obj free pose
		
		#to check if a pose is too close to or blocked by the furnitures in the room 
		dist_to_obstacles = 0.45 #minimum disatnce to the furnitures in the room
		index_2 = 0
		while index_2 < len(parent_obj_checked_grasp_base_pose_list):
			index_3 = 0
			while index_3 < len(furniture_geometry_list):
				th = math.atan((parent_obj_checked_grasp_base_pose_list[index_2].y - furniture_geometry_list[index_3].pose.y) / (parent_obj_checked_grasp_base_pose_list[index_2].x - furniture_geometry_list[index_3].pose.x))
				if parent_obj_checked_grasp_base_pose_list[index_2].x < furniture_geometry_list[index_3].pose.x and parent_obj_checked_grasp_base_pose_list[index_2].y > furniture_geometry_list[index_3].pose.y:
					th = math.pi + th
				if parent_obj_checked_grasp_base_pose_list[index_2].x < furniture_geometry_list[index_3].pose.x and parent_obj_checked_grasp_base_pose_list[index_2].y < furniture_geometry_list[index_3].pose.y:
					th = -math.pi + th
				delta_x = math.sqrt((parent_obj_checked_grasp_base_pose_list[index_2].x - furniture_geometry_list[index_3].pose.x) ** 2 + (parent_obj_checked_grasp_base_pose_list[index_2].y - furniture_geometry_list[index_3].pose.y) ** 2) * math.cos(th - furniture_geometry_list[index_3].pose.theta)
				delta_y = math.sqrt((parent_obj_checked_grasp_base_pose_list[index_2].x - furniture_geometry_list[index_3].pose.x) ** 2 + (parent_obj_checked_grasp_base_pose_list[index_2].y - furniture_geometry_list[index_3].pose.y) ** 2) * math.sin(th - furniture_geometry_list[index_3].pose.theta)
				if (delta_x <= -(furniture_geometry_list[index_3].w / 2.0 + dist_to_obstacles) or delta_x >= (furniture_geometry_list[index_3].w / 2.0 + dist_to_obstacles)) or (delta_y <= -(furniture_geometry_list[index_3].l / 2.0 + dist_to_obstacles) or delta_y >= (furniture_geometry_list[index_3].l / 2.0 + dist_to_obstacles)):
					index_3 += 1
				else:
					break
			if index_3 == len(furniture_geometry_list):
				obstacle_checked_grasp_base_pose_list.append(parent_obj_checked_grasp_base_pose_list[index_2])
			index_2 += 1
		#rospy.loginfo(obstacle_checked_grasp_base_pose_list)

		if obstacle_checked_grasp_base_pose_list: 
			
			#to check if a pose is too near to or blocked by the wall
			data = getMapClient() #get occupancy grid map
			#rospy.loginfo(data.map.info)
			dist_to_walls = 0.5 #set the minimum distance to the walls
			threshold = 20 #>20:occupied
			step_angle = 5.0 #(360 / step_angle) points will be put around the robot for the wall check

			index_4 = 0
			while index_4 < len(obstacle_checked_grasp_base_pose_list):
				map_index_list = list()
				n = 0
				while n < int(360.0 / step_angle):
					wall_check_point_x = obstacle_checked_grasp_base_pose_list[index_4].x + dist_to_walls * math.cos(n * step_angle / 180.0 * math.pi)
					wall_check_point_y = obstacle_checked_grasp_base_pose_list[index_4].y + dist_to_walls * math.sin(n * step_angle / 180.0 * math.pi)
					map_index = int((wall_check_point_y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((wall_check_point_x - data.map.info.origin.position.x) / data.map.info.resolution)
					map_index_list.append(map_index)
					n += 1
				
				map_index = int((obstacle_checked_grasp_base_pose_list[index_4].y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((obstacle_checked_grasp_base_pose_list[index_4].x - data.map.info.origin.position.x) / data.map.info.resolution)
				map_index_list.append(map_index)
				#rospy.loginfo(map_index_list)

				index_5 = 0
				while index_5 < len(map_index_list):
					if -1 < data.map.data[map_index_list[index_5]] < threshold:
						index_5 += 1
					else:
						break
				if index_5 == len(map_index_list):
					wall_checked_grasp_base_pose_list.append(obstacle_checked_grasp_base_pose_list[index_4])
				index_4 += 1
	
	return 	wall_checked_grasp_base_pose_list 			
	#rospy.loginfo(wall_checked_grasp_base_pose_list)
	

def handle_symbol_grounding_grasp_base_region(req):
	
	'''
	#get the robot's current pose from tf
	rb_pose = Pose2D()
	listener = tf.TransformListener()
	listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(4.0)) #wait for 4secs for the coordinate to be transformed
	(trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
	rb_pose.x = trans[0]
	rb_pose.y = trans[1]
	rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
	rb_pose.theta = rb_pose_rpy[2]
	rospy.sleep(0.5)
	#rospy.loginfo(rb_pose)
	'''
	rb_pose = Pose2D()
	rb_pose.x = -1.06
	rb_pose.y = -1.08
	rb_pose.theta = 0.0
		
	#predefined membership function
	mf1_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf1_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125, 0]
	mf1_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

	'''
	mf2_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf2_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
	mf2_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

	mf3_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf3_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
	mf3_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]
	'''


	#transfrom the parent obj data from database 
	target_obj_x = req.target_obj_pose.position.x
	target_obj_y = req.target_obj_pose.position.y
	target_obj_rpy = tf.transformations.euler_from_quaternion([req.target_obj_pose.orientation.x, req.target_obj_pose.orientation.y, req.target_obj_pose.orientation.z, req.target_obj_pose.orientation.w])
	target_obj_th = target_obj_rpy[2]
	#rospy.loginfo(target_obj_th)

	robot_base_pose_x = rb_pose.x
	robot_base_pose_y = rb_pose.y
	robot_base_pose_th = rb_pose.theta

	parent_obj_x = req.parent_obj_geometry.pose.position.x
	parent_obj_y = req.parent_obj_geometry.pose.position.y
	parent_obj_rpy = tf.transformations.euler_from_quaternion([req.parent_obj_geometry.pose.orientation.x, req.parent_obj_geometry.pose.orientation.y, req.parent_obj_geometry.pose.orientation.z, req.parent_obj_geometry.pose.orientation.w])
	parent_obj_th = parent_obj_rpy[2]
	parent_obj_l = req.parent_obj_geometry.l
	parent_obj_w = req.parent_obj_geometry.w
	parent_obj_h = req.parent_obj_geometry.h


	'''
	#get furniture list from the knowledge base (part 2)

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
	#transfrom furniture geometry data from database 
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
	



	#calculate the reachability

	#calculate the best grippier pose from the robot's current base pose 
	best_gripper_pose_x = robot_base_pose_x - 0.8 * math.cos(robot_base_pose_th) + 0.1 * math.sin(robot_base_pose_th)
	best_gripper_pose_y = robot_base_pose_y - 0.8 * math.sin(robot_base_pose_th) - 0.1 * math.cos(robot_base_pose_th) 

	#calculate the difference between the best gripper pose and the target obj pose
	th = math.atan(target_obj_y - best_gripper_pose_y / target_obj_x - best_gripper_pose_x)
	if target_obj_x < best_gripper_pose_x and target_obj_y > best_gripper_pose_y:
		th += math.pi
	elif target_obj_x < best_gripper_pose_x and target_obj_y < best_gripper_pose_y:
		th -= math.pi
	delta_x = math.sqrt((target_obj_x - best_gripper_pose_x) ** 2 + (target_obj_y - best_gripper_pose_y) ** 2) * math.cos(th - robot_base_pose_th) 
	delta_y = math.sqrt((target_obj_x - best_gripper_pose_x) ** 2 + (target_obj_y - best_gripper_pose_y) ** 2) * math.sin(th - robot_base_pose_th)
	th =  math.atan((rb_pose.y - target_obj_y) / (rb_pose.x - target_obj_x))
	if rb_pose.x < target_obj_x and rb_pose.y > target_obj_y:
		th += math.pi
	elif rb_pose.x < target_obj_x and rb_pose.y < target_obj_y:
		th -= math.pi
	delta_th = robot_base_pose_th - th
	if delta_th < 0:
		delta_th = -1.0 * delta_th

	#if target obj pose is out side hdz, reach = 0
	if delta_x > 0.1 or delta_x < -0.15:
		reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		reach = 0
	elif delta_th < -10.0 / 180.0 * math.pi or delta_th > 10.0 / 180 * math.pi:
		reach = 0
	else:
	#calculate membership value according to the membership functions
		index_x = int(round(delta_x / 0.025 + 6))
		index_y = int(round(delta_y / 0.025 + 6))
		index_th = int(round(delta_th / (1.0 / 180.0 * math.pi) + 10)) 
		member_x = mf1_x[index_x]
		member_y = mf1_y[index_y]
		member_th = mf1_th[index_th]
		#Apply the fuzzy rule.
		reach = min(member_x, member_y, member_th)
		#rospy.loginfo(reach)



	#calculate two lists of candidate base poses for grasping 	
	#the first candidate pose list
	step_angle_1 = 5.0 / 180.0 * math.pi #360 / step_angle of candidate poses will be put around the target obj
	dist_1 = 0.8 #distance to the target obj
	grasp_base_pose_list_1 = list()
	grasp_base_pose_list_1 = getRobotBasePoseList(step_angle_1, dist_1, rb_pose, target_obj_x, target_obj_y)
	#rospy.loginfo(grasp_base_pose_list)

	#calculate another list of candidate grasping poses with lower reachability
	step_angle_2 = 5.0 / 180.0 * math.pi
	dist_2 = 0.9
	grasp_base_pose_list_2 = list()
	grasp_base_pose_list_2 = getRobotBasePoseList(step_angle_2, dist_2, rb_pose, target_obj_x, target_obj_y)
	#rospy.loginfo(grasp_base_pose_list) 

	
	#optimisation
	obstacle_check = 1
	#obstacle check
	grasp_base_pose_list = obstacleCheck(grasp_base_pose_list_1, parent_obj_x, parent_obj_y, parent_obj_th, parent_obj_w, parent_obj_l, furniture_geometry_list)

	grasp_base_pose = Pose2D()
	#rospy.loginfo(grasp_base_pose_list)
	if grasp_base_pose_list:
		obstacle_check = 0
		grasp_base_pose = grasp_base_pose_list[0]

	else:
		#obstacle check for the grasping poses with lower reachability when no valid pose can be found from the first candidate pose list
		grasp_base_pose_list = obstacleCheck(grasp_base_pose_list_2, parent_obj_x, parent_obj_y, parent_obj_th, parent_obj_w, parent_obj_l, furniture_geometry_list)
	
	#rospy.loginfo(grasp_base_pose_list)
	#check if there is valid pose
	if grasp_base_pose_list:

		obstacle_check = 0
		grasp_base_pose = grasp_base_pose_list[0]		
	

	#calculate R
	min_dist = 0.40 #set the minimum distance to obstacles
	hdz_size = 0.05
	dist_list = list()
	index = 0
	while index < len(furniture_geometry_list):
		th = math.atan((grasp_base_pose.y - furniture_geometry_list[index].pose.y) / (grasp_base_pose.x - furniture_geometry_list[index].pose.x))
		if grasp_base_pose.x < furniture_geometry_list[index].pose.x and grasp_base_pose.y > furniture_geometry_list[index].pose.y:
			th = math.pi + th
		if grasp_base_pose.x < furniture_geometry_list[index].pose.x and grasp_base_pose.y < furniture_geometry_list[index].pose.y:
			th = -math.pi + th
		delta_x = math.sqrt((grasp_base_pose.x - furniture_geometry_list[index].pose.x) ** 2 + (grasp_base_pose.y - furniture_geometry_list[index].pose.y) ** 2) * math.cos(th - furniture_geometry_list[index].pose.theta)
		delta_y = math.sqrt((grasp_base_pose.x - furniture_geometry_list[index].pose.x) ** 2 + (grasp_base_pose.y - furniture_geometry_list[index].pose.y) ** 2) * math.sin(th - furniture_geometry_list[index].pose.theta)
		delta_x = abs(delta_x)
		delta_y = abs(delta_y)
		dist = max((delta_x - furniture_geometry_list[index].w / 2.0), (delta_y - furniture_geometry_list[index].l / 2.0))
		dist_list.append(dist)
		index += 1
	R = min(min(dist_list) - min_dist, hdz_size)
	

		
	#return value
	return obstacle_check, reach, grasp_base_pose, R





	



def symbol_grounding_grasp_base_region_server():
	rospy.init_node('symbol_grounding_grasp_base_region_server')
	s = rospy.Service('symbol_grounding_grasp_base_region', SymbolGroundingGraspBaseRegion, handle_symbol_grounding_grasp_base_region)
	print "Ready to receive requests."
	rospy.spin()




if __name__ == "__main__":
	symbol_grounding_grasp_base_region_server()
