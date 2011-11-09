#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_scenarios
# \note
#   ROS package name: cob_generic_states
#
# \author
#   Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: Aug 2011
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
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
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
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
roslib.load_manifest('cob_generic_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection.msg import *
from cob_object_detection.srv import *

## Detect state
#
# This state will try to detect an object.
class detect_object(smach.State):
	def __init__(self,object_name = "",max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded','retry','no_more_retries','failed'],
			input_keys=['object_name'],
			output_keys=['object'])

		self.object_list = DetectionArray()
		self.max_retries = max_retries
		self.retries = 0
		self.object_name = object_name
		
		self.torso_poses = []
		self.torso_poses.append("back_right_extreme")
		self.torso_poses.append("back_extreme")
		self.torso_poses.append("back_left_extreme")

	def execute(self, userdata):
		# determine object name
		if self.object_name != "":
			object_name = self.object_name
		elif type(userdata.object_name) is str:
			object_name = userdata.object_name
		else: # this should never happen
			rospy.logerr("Invalid userdata 'object_name'")
			self.retries = 0
			return 'failed'
	
		# check if maximum retries reached
		if self.retries > self.max_retries:
			self.retries = 0
			return 'no_more_retries'
		
		# move sdh as feedback
		sss.move("sdh","cylclosed",False)
		
		# make the robot ready to inspect the scene
		if self.retries == 0: # only move arm, sdh and head for the first try
			sss.say(["I will now search for the " + object_name + "."],False)
			handle_arm = sss.move("arm","folded-to-look_at_table",False)
			handle_torso = sss.move("torso","shake",False)
			handle_head = sss.move("head","back",False)
			handle_arm.wait()
			handle_head.wait()
			handle_torso.wait()
		handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) # have an other viewing point for each retry
		
		# move sdh as feedback
		sss.move("sdh","home",False)
	
		# check if object detection service is available
		try:
			rospy.wait_for_service('/object_detection/detect_object',10)
		except rospy.ROSException, e:
			print "Service not available: %s"%e
			self.retries = 0 # no object found within min_dist start value
			return 'failed'

		# call object detection service
		try:
			detector_service = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
			req = DetectObjectsRequest()
			req.object_name.data = object_name
			res = detector_service(req)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'
			
		# check for no objects
		if len(res.object_list.detections) <= 0:
			rospy.logerr("No objects found")
			self.retries += 1
			return 'retry'
		
		# select nearest object in x-y-plane in head_camera_left_link
		min_dist = 2 # start value in m
		obj = Detection()
		for item in res.object_list.detections:
			dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
			if dist < min_dist:
				min_dist = dist
				obj = copy.deepcopy(item)
		
		# check if an object could be found within the min_dist start value
		if obj.header.frame_id == "":
			self.retries += 1
			return 'retry'

		#check if label of object fits to requested object_name
		if obj.label != object_name:
			sss.say(["The object name doesn't fit."],False)
			self.retries += 1
			return 'retry'

		# we succeeded to detect an object
		userdata.object = obj
		self.retries = 0
		return 'succeeded'
