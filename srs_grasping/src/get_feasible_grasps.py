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
#   Implements a service that returns all the feasible grasping configurations for a given object_id and target pose.
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
import tf
import rospy
import time
import grasping_functions 

from srs_grasping.srv import *
from srs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from kinematics_msgs.srv import *


class get_feasible_grasps():

	def __init__(self):

		rospy.loginfo("Waiting /arm_kinematics/get_constraint_aware_ik service...");
		rospy.wait_for_service('/arm_kinematics/get_constraint_aware_ik')
		rospy.loginfo("/arm_kinematics/get_constraint_aware_ik is ready.");

		rospy.loginfo("Waiting /get_db_grasps service...");
		rospy.wait_for_service('/get_db_grasps')
		self.client = rospy.ServiceProxy('/get_db_grasps', GetDBGrasps)
		rospy.loginfo("/get_feasible_grasps service is ready.");

		self.ik_loop_reply = 1;
		self.finger_correction = -1;
		self.current_joint_configuration = []
		self.finger_positions = []

		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		self.tf = rospy.Subscriber("/tf", tf.msg.tfMessage, self.get_finger_positions);
		self.arm_state = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state);



	def get_feasible_grasps(self, req):
		x = time.time();
		rospy.loginfo("/get_feasible_grasps service has been called...");

		obj_id = req.object_id;
		obj_pose = req.object_pose;
		pregrasp_offsets = req.pregrasp_offsets;

		req = GetDBGraspsRequest();
		req.object_id = obj_id;
		grasp_configuration = (self.client(req)).grasp_configuration;

		#current_joint_configuration
		while self.arm_state.get_num_connections() == 0:
			time.sleep(0.3);
			continue;


		rotacion = grasping_functions.graspingutils.rotation_matrix(obj_pose);

		resp = GetFeasibleGraspsResponse();
		resp.feasible_grasp_available = False;
		resp.grasp_configuration = [];

		for i in range(0,len(grasp_configuration)):
			pre_trans = rotacion * grasping_functions.graspingutils.matrix_from_pose(grasp_configuration[i].pre_grasp.pose);
			grasp_trans = rotacion *  grasping_functions.graspingutils.matrix_from_pose(grasp_configuration[i].grasp.pose);

			t = tf.transformations.translation_from_matrix(pre_trans);
			q = tf.transformations.quaternion_from_matrix(pre_trans);
			tg = tf.transformations.translation_from_matrix(grasp_trans);
			qg = tf.transformations.quaternion_from_matrix(grasp_trans);


			pre = PoseStamped();
			pre.header.stamp = rospy.Time.now();
			pre.header.frame_id = "/base_link";
			pre.pose.position.x = t[0];
			pre.pose.position.y = t[1];
			pre.pose.position.z = t[2];
			pre.pose.orientation.x = q[0];
			pre.pose.orientation.y = q[1];
			pre.pose.orientation.z = q[2];
			pre.pose.orientation.w = q[3];

			g = PoseStamped();
			g.header.stamp = rospy.Time.now();
			g.header.frame_id = "/base_link";
			g.pose.position.x = tg[0];
			g.pose.position.y = tg[1];
			g.pose.position.z = tg[2];
			g.pose.orientation.x = qg[0];
			g.pose.orientation.y = qg[1];
			g.pose.orientation.z = qg[2];
			g.pose.orientation.w = qg[3];
			
			category = grasping_functions.graspingutils.get_grasp_category(pre.pose.position, g.pose.position);
			pre.pose = grasping_functions.graspingutils.set_pregrasp_offsets(category, pre.pose, pregrasp_offsets);

			sol = False;
			for w in range(0,self.ik_loop_reply):
				(pre_grasp_conf, error_code) = grasping_functions.graspingutils.callIKSolver(self.current_joint_configuration, pre);				
				if(error_code.val == error_code.SUCCESS):
					for k in range(0,self.ik_loop_reply):
						(grasp_conf, error_code) = grasping_functions.graspingutils.callIKSolver(pre_grasp_conf, g);
						if(error_code.val == error_code.SUCCESS):
							new_valid_grasp = FeasibleGrasp();
							new_valid_grasp.sdh_joint_values = grasp_configuration[i].sdh_joint_values;
							new_valid_grasp.target_link = "/sdh_palm_link"
							new_valid_grasp.grasp = g;
							new_valid_grasp.pre_grasp = pre;
							new_valid_grasp.category = category;
	
							sol = grasping_functions.graspingutils.valid_grasp(category);
							while len(self.finger_positions) != 3:
								time.sleep(0.3);
								continue;

							if sol and (not  self.checkCollisions(self.finger_positions, new_valid_grasp, obj_pose)):
								resp.feasible_grasp_available = True;
								resp.grasp_configuration.append(new_valid_grasp);
								break;


					if sol:
						break;
			#for
		#for

		#order grasps
		if len(resp.grasp_configuration) > 0:
			(resp.grasp_configuration).sort();

		rospy.loginfo(str(len(resp.grasp_configuration))+" valid grasps for this pose.");	
		rospy.loginfo("/get_feasible_grasps call has finished.");
		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";
		return resp;
	 


	def __cmp__(self, other):
		if self.grasp.position.z < other.grasp.position.z :
			rst = 1
		elif self.grasp.position.z > other.grasp.position.z :
			rst = -1
		else :
			rst = 0

		return rst



	def get_joint_state(self, msg):
		self.current_joint_configuration = list(msg.desired.positions);



	def get_finger_positions(self, msg):
		fing_pos = [];

		for link in msg.transforms:
			if link.child_frame_id == "/sdh_finger_13_link" or link.child_frame_id == "/sdh_finger_23_link" or link.child_frame_id == "/sdh_thumb_3_link":
				fp = PoseStamped();
				fp.header.frame_id = link.header.frame_id;
				fp.pose.position.x = link.transform.translation.x;
				fp.pose.position.y = link.transform.translation.y;
				fp.pose.position.z = link.transform.translation.z;
				fp.pose.orientation.x = link.transform.rotation.x;
				fp.pose.orientation.y = link.transform.rotation.y;
				fp.pose.orientation.z = link.transform.rotation.z;
				fp.pose.orientation.w = link.transform.rotation.w;
				fing_pos.append(fp);

				if len(fing_pos) == 3:
					self.finger_positions = fing_pos;
					break;



	def transform_finger_positions(self, fpositions, g):
		response = [];
		for i in range(0, len(fpositions)):
			ps = PoseStamped();
			ps.header.stamp = self.listener.getLatestCommonTime("/sdh_palm_link", fpositions[i].header.frame_id);
			ps = self.listener.transformPose("/sdh_palm_link", fpositions[i]);

			#transform to object position
			aux_matrix = grasping_functions.graspingutils.matrix_from_pose(g) * grasping_functions.graspingutils.matrix_from_pose(ps.pose);
			t = tf.transformations.translation_from_matrix(aux_matrix);
			q = tf.transformations.quaternion_from_matrix(aux_matrix);

			res = PoseStamped();
			res.header.stamp = rospy.Time.now();
			res.header.frame_id = "/base_link";
			res.pose.position.x = t[0];
			res.pose.position.y = t[1];
			res.pose.position.z = t[2];
			res.pose.orientation.x = q[0];
			res.pose.orientation.y = q[1];
			res.pose.orientation.z = q[2];
			res.pose.orientation.w = q[3];
			response.append(res);

		return response;



	def checkCollisions(self, fpositions, new_valid_grasp, obj_pose):	
		self.finger_correction = (0, -0.0685)[new_valid_grasp.category == "TOP"];
		finger_pos = self.transform_finger_positions(fpositions, new_valid_grasp.grasp.pose);
		
		for fp in finger_pos:
			if obj_pose.position.z > (fp.pose.position.z + self.finger_correction):
				return True;

		return False;



	def get_feasible_grasps_server(self):
		s = rospy.Service('/get_feasible_grasps', GetFeasibleGrasps, self.get_feasible_grasps);


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('get_feasible_grasps');
	SCRIPT = get_feasible_grasps();
	SCRIPT.get_feasible_grasps_server();
	rospy.spin();
