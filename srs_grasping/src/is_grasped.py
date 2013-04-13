#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2013 \n
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
# \date Date of creation: April 2013
#
# \brief
#   Service to check if the hand is grasping something.
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
import rospy
import time

from cob_srvs.srv import Trigger, TriggerResponse
from schunk_sdh.msg import TactileSensor

class is_grasped():

	def __init__(self):
		self.f1 = []; self.f2 = []; self.f3 = [];
		self.sdh_state = rospy.Subscriber("/sdh_controller/tactile_data", TactileSensor, self.get_sdh_state)
		rospy.loginfo("/srs_grasping/is_grasped service is ready.");


	def get_sdh_state(self, msg):
		self.f1 = msg.tactile_matrix[0].tactile_array + msg.tactile_matrix[1].tactile_array;
		self.f2 = msg.tactile_matrix[2].tactile_array + msg.tactile_matrix[3].tactile_array;
		self.f3 = msg.tactile_matrix[4].tactile_array + msg.tactile_matrix[5].tactile_array;

	def is_grasped(self, request):
		rospy.loginfo("/srs_grasping/is_grasped service has been called...");

		res = TriggerResponse()

		while self.sdh_state.get_num_connections() == 0:
			rospy.loginfo("[srs_grasping/is_grasped] Waiting for /sdh_controller/state connections...")
			time.sleep(0.3);
			continue;

		size = len(self.f1)
		if size == 0:
			res.success.data = False
			res.error_message.data = "no data received"
			rospy.loginfo("[srs_grasping/is_grasped]: %s", res.error_message.data);
			return res;

		f1=0; f2=0; f3=0;
		for i in range(0,size):
			if self.f1[i] > 0: #> 500
				f1+=1;
			if self.f2[i] > 0:
				f2+=1;
			if self.f3[i] > 0:
				f3+=1;

		num_fingers=0;
		if f1>0:
			num_fingers+=1;
		if f2>0:
			num_fingers+=1;
		if f3>0:
			num_fingers+=1;

		if num_fingers>=1: #>=2
			res.success.data = True
			res.error_message.data = "Number of fingers touching the object: ",num_fingers
		else:
			res.success.data = False
			res.error_message.data = "Fingers are not touching the object"

		rospy.loginfo("[srs_grasping/is_grasped]: %s", res.error_message.data);
		return res;


	def is_grasped_server(self):
		s = rospy.Service('/srs_grasping/is_grasped', Trigger, self.is_grasped);


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('is_grasped_node');
	SCRIPT = is_grasped();
	SCRIPT.is_grasped_server();
	rospy.spin();
