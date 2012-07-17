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
# \date Date of creation: Abril 2012
#
# \brief
#   Implements a service that returns a group of pregrasp positions.
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
import grasping_functions 

from srs_grasping.srv import *
from srs_msgs.msg import *
from tf.transformations import *
from geometry_msgs.msg import *


class get_pregrasps():

	def __init__(self):
		self.ik_loop_reply = 1

		rospy.loginfo("Waiting /get_db_grasps service...");
		rospy.wait_for_service('/get_db_grasps')
		self.client = rospy.ServiceProxy('/get_db_grasps', GetDB_Grasps)
		rospy.loginfo("/get_pregrasps service is ready.");
		print "---------------------------------------------------------------------------";

	def get_pregrasps(self, req):
		x = time.time();
		rospy.loginfo("/get_pregrasps service has been called...");

		obj_id = req.object_id;
		obj_pose = req.object_pose;
		pregrasp_offsets = req.pregrasp_offsets;
		num_configurations = (1, req.num_configurations)[req.num_configurations>0];
		

		req = GetDB_GraspsRequest();
		req.object_id = obj_id;
		grasp_configuration = (self.client(req)).grasp_configuration;


		rotacion = grasping_functions.graspingutils.rotation_matrix(obj_pose);

		resp = GetPreGraspResponse();
		resp.side = []
		resp.mside = []
		resp.top = []
		resp.front = []

		for i in range(0,len(grasp_configuration)):
			pre_trans = rotacion * grasping_functions.graspingutils.matrix_from_pose(grasp_configuration[i].pre_grasp.pose);
			grasp_trans = rotacion *  grasping_functions.graspingutils.matrix_from_pose(grasp_configuration[i].grasp.pose);

			t = translation_from_matrix(pre_trans);
			q = quaternion_from_matrix(pre_trans);
			tg = translation_from_matrix(grasp_trans);
			qg = quaternion_from_matrix(grasp_trans);

			pre = Pose();
			pre.position.x = t[0];
			pre.position.y = t[1];
			pre.position.z = t[2];
			pre.orientation.x = q[0];
			pre.orientation.y = q[1];
			pre.orientation.z = q[2];
			pre.orientation.w = q[3];

			g = Pose();
			g.position.x = tg[0];
			g.position.y = tg[1];
			g.position.z = tg[2];
			g.orientation.x = qg[0];
			g.orientation.y = qg[1];
			g.orientation.z = qg[2];
			g.orientation.w = qg[3];


			category = grasping_functions.graspingutils.get_grasp_category(pre.position, g.position);
			pre = grasping_functions.graspingutils.set_pregrasp_offsets(category, pre, pregrasp_offsets);


			aux = DB_Grasp();
			aux.object_id = obj_id;
			aux.hand_type = "SDH";
			aux.sdh_joint_values = grasp_configuration[i].sdh_joint_values;
			aux.pre_grasp.pose = pre;
			aux.grasp.pose = g;
			aux.category = grasping_functions.graspingutils.get_grasp_category(pre.position, g.position);

			if aux.category == "TOP":
				if len(resp.top) < num_configurations:
					resp.top.append(aux);
			elif aux.category == "FRONT":
				if len(resp.front) < num_configurations:
					resp.front.append(aux);
			elif aux.category == "SIDE":
				if len(resp.side) < num_configurations:
					resp.side.append(aux);
			elif aux.category == "-SIDE":
				if len(resp.mside) < num_configurations:
					resp.mside.append(aux);
			else:
				continue

			if len(resp.top)==num_configurations and len(resp.side)==num_configurations and len(resp.mside)==num_configurations and len(resp.front)==num_configurations:
				break;

		rospy.loginfo("/get_pregrasps call has finished.");
		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";
		return resp;


	def get_pregrasps_server(self):
		s = rospy.Service('/get_pregrasps', GetPreGrasp, self.get_pregrasps);


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('get_pregrasps');
	SCRIPT = get_pregrasps();
	SCRIPT.get_pregrasps_server();
	rospy.spin();
