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
# Grounding deliver object commands
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




def getMapClient(): #read map from navigation service

	try:
		reqMap = rospy.ServiceProxy('static_map', GetMap)	
		res = reqMap()
		return res
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e



def obstacleCheck(dbp, fgl): 
	deliver_base_pose = dbp
	furniture_geometry_list = fgl
	obstacle_checked_deliver_base_pose = Pose2D()
	wall_checked_deliver_base_pose = Pose2D()

	#obstacle check
	dist_to_obstacles = 0.4  #set the minimum distance to the household furnitures
	index = 0
	while index < len(furniture_geometry_list):
		th = math.atan((deliver_base_pose.y - furniture_geometry_list[index].pose.y) / (deliver_base_pose.x - furniture_geometry_list[index].pose.x))
		if deliver_base_pose.x < furniture_geometry_list[index].pose.x and deliver_base_pose.y > furniture_geometry_list[index].pose.y:
			th = math.pi + th
		if deliver_base_pose.x < furniture_geometry_list[index].pose.x and deliver_base_pose.y < furniture_geometry_list[index].pose.y:
			th = -math.pi + th
		dist = math.sqrt((deliver_base_pose.x - furniture_geometry_list[index].pose.x) ** 2 + (deliver_base_pose.y - furniture_geometry_list[index].pose.y) ** 2)
		delta_x = dist * math.cos(th - furniture_geometry_list[index].pose.theta)
		delta_y = dist * math.sin(th - furniture_geometry_list[index].pose.theta)
		if (delta_x <= -(furniture_geometry_list[index].w / 2.0 + dist_to_obstacles) or delta_x >= (furniture_geometry_list[index].w / 2.0 + dist_to_obstacles)) or (delta_y <= -(furniture_geometry_list[index].l / 2.0 + dist_to_obstacles) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + dist_to_obstacles)):
			index += 1
		else:
			break
	if index == len(furniture_geometry_list):
		obstacle_checked_deliver_base_pose = deliver_base_pose
		
	print obstacle_checked_deliver_base_pose
	if obstacle_checked_deliver_base_pose: #check if there is a obstacle free pose.
			
		#wall check
		data = getMapClient() #get occupancy grid map from the navigation serves 	
		dist_to_walls = 0.5 #set the minimum distance to the walls
		threshold = 10.0 #set the threshold to decide if a pose is occupaied. >threshold:occupied.
		step_angle = 10.0 #set the step angle for putting points around the robot for the wall check (360 / step_angle points will be used)
		map_index_list = list()

		n = 0
		while n < int(360.0 / step_angle):
			wall_check_point_x = obstacle_checked_deliver_base_pose.x + dist_to_walls * math.cos(n * step_angle / 180.0 * math.pi)
			wall_check_point_y = obstacle_checked_deliver_base_pose.y + dist_to_walls * math.sin(n * step_angle / 180.0 * math.pi)	
			map_index = int((wall_check_point_y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((wall_check_point_x - data.map.info.origin.position.x) / data.map.info.resolution)
			map_index_list.append(map_index)
			n += 1
				
		map_index = int((obstacle_checked_deliver_base_pose.y - data.map.info.origin.position.y) / data.map.info.resolution) * data.map.info.width + int((obstacle_checked_deliver_base_pose.x - data.map.info.origin.position.x) / data.map.info.resolution)
		map_index_list.append(map_index)

		index = 0
		while index < len(map_index_list):
			print data.map.data[map_index_list[index]]
			if -1 < data.map.data[map_index_list[index]] < threshold:
				index += 1
			else:
				break
		print index
		if index == len(map_index_list):
			wall_checked_deliver_base_pose = obstacle_checked_deliver_base_pose
		
	return 	wall_checked_deliver_base_pose			




#calculate deliver base region
def handle_symbol_grounding_deliver_base_region(req):


	#transform from knowledge base data to function useable data  	
	parent_obj_x = req.parent_obj_geometry.pose.position.x
	parent_obj_y = req.parent_obj_geometry.pose.position.y
	parent_obj_rpy = tf.transformations.euler_from_quaternion([req.parent_obj_geometry.pose.orientation.x, req.parent_obj_geometry.pose.orientation.y, req.parent_obj_geometry.pose.orientation.z, req.parent_obj_geometry.pose.orientation.w])
	parent_obj_th = parent_obj_rpy[2]
	parent_obj_l = req.parent_obj_geometry.l
	parent_obj_w = req.parent_obj_geometry.w
	parent_obj_h = req.parent_obj_geometry.h
	
	#transfrom furniture geometry list
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
	
	#variable structure define
	deliver_base_pose = Pose2D()
	deliver_base_pose_1 = Pose2D()
	deliver_base_pose_2 = Pose2D()

	#fix angle problem
	if parent_obj_th < 0:
		parent_obj_th += 2.0 * math.pi
	elif parent_obj_th > 2.0 * math.pi:
		parent_obj_th -= 2.0 * math.pi

	#calculate deliver base poses
	rb_distance = 0.5
	deliver_base_pose_1.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + 0.3 * parent_obj_l * math.sin(parent_obj_th)
	deliver_base_pose_1.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - 0.3 * parent_obj_l * math.cos(parent_obj_th)
	deliver_base_pose_1.theta = parent_obj_th
	if deliver_base_pose_1.theta > math.pi:
		deliver_base_pose_1.theta -= 2.0 * math.pi
	elif deliver_base_pose_1.theta < -math.pi:
		deliver_base_pose_1.theta += 2.0 * math.pi

	deliver_base_pose_2.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - 0.3 * parent_obj_l * math.sin(parent_obj_th)
	deliver_base_pose_2.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + 0.3 * parent_obj_l * math.cos(parent_obj_th)
	deliver_base_pose_2.theta = parent_obj_th
	if deliver_base_pose_2.theta > math.pi:
		deliver_base_pose_2.theta -= 2.0 * math.pi
	elif deliver_base_pose_2.theta < -math.pi:
		deliver_base_pose_2.theta += 2.0 * math.pi
	
	#obstacle check
	obstacle_checked_deliver_base_pose_1 = obstacleCheck(deliver_base_pose_1, furniture_geometry_list)
	obstacle_checked_deliver_base_pose_2 = obstacleCheck(deliver_base_pose_2, furniture_geometry_list)

	#choose deliver base pose
	if obstacle_checked_deliver_base_pose_1 and obstacle_checked_deliver_base_pose_2:

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
		
		dist_1 = (rb_pose.x - obstacle_checked_deliver_base_pose_1.x) ** 2 + (rb_pose.y - obstacle_checked_deliver_base_pose_1.y) ** 2
		dist_2 = (rb_pose.x - obstacle_checked_deliver_base_pose_2.x) ** 2 + (rb_pose.y - obstacle_checked_deliver_base_pose_2.y) ** 2
		if dist_1 > dist_2:
			deliver_base_pose = obstacle_checked_deliver_base_pose_2
		else:
			deliver_base_pose = obstacle_checked_deliver_base_pose_1

	elif obstacle_checked_deliver_base_pose_1:
		deliver_base_pose = obstacle_checked_deliver_base_pose_1
	elif obstacle_checked_deliver_base_pose_2:
		deliver_base_pose = obstacle_checked_deliver_base_pose_2
	else:
		print "no valid deliver pose."

	R = 0.0
	if deliver_base_pose:
		#calculate R
		min_dist = 0.40 #set the minimum distance to obstacles
		deliver_region_size = 0.1
		dist_list = list()
		index = 0
		while index < len(furniture_geometry_list):
			th = math.atan((deliver_base_pose.y - furniture_geometry_list[index].pose.y) / (deliver_base_pose.x - furniture_geometry_list[index].pose.x))
			if deliver_base_pose.x < furniture_geometry_list[index].pose.x and deliver_base_pose.y > furniture_geometry_list[index].pose.y:
				th = math.pi + th
			if deliver_base_pose.x < furniture_geometry_list[index].pose.x and deliver_base_pose.y < furniture_geometry_list[index].pose.y:
				th = -math.pi + th
			d = math.sqrt((deliver_base_pose.x - furniture_geometry_list[index].pose.x) ** 2 + (deliver_base_pose.y - furniture_geometry_list[index].pose.y) ** 2)
			delta_x = d * math.cos(th - furniture_geometry_list[index].pose.theta)
			delta_y = d * math.sin(th - furniture_geometry_list[index].pose.theta)
			delta_x = abs(delta_x)
			delta_y = abs(delta_y)
			dist = max((delta_x - furniture_geometry_list[index].w / 2.0), (delta_y - furniture_geometry_list[index].l / 2.0))
			dist_list.append(dist)
			index += 1
		R = min(min(dist_list) - min_dist, deliver_region_size)

	return deliver_base_pose, R



def symbol_grounding_deliver_base_region_server():
	rospy.init_node('symbol_grounding_deliver_base_region_server')
	s = rospy.Service('symbol_grounding_deliver_base_region', SymbolGroundingDeliverBaseRegion, handle_symbol_grounding_deliver_base_region)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_deliver_base_region_server()


