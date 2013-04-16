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
from srs_msgs.msg import GraspingErrorCodes

class get_feasible_grasps():

	def __init__(self):

		self.ik_loop_reply = 1;
		self.finger_correction = -1;
		self.cjc = []
		self.cylopen_position = [[[0.09121056454806604, 0.03300469295835018, 0.1386980387655565],[0.01977019790965633, 6.428768666637955e-07, 0.9998045500082638, -3.251111695101345e-05]], [[0.01905, 0.033, 0.091],[0.47359137769338583, 1.5399994595997778e-05, 0.880744688270724, -2.863959117221133e-05]], [[-0.11021758805551472, 0.0, 0.13877808854293133],[0.0, -0.019200739055054376, 0.0, 0.9998156488171905]]]
		self.spheropen_position = [[[0.04967501468386526, 0.08596897096059274, 0.15214447310255375],[-0.11284765597435997, -0.06510645561927879, 0.8587963331213594, -0.49547493800907255]], [[0.01905, 0.033, 0.091],[0.3316048124188912, 0.19131645949114948, 0.8001899672091825, -0.4616625142744111]], [[-0.0992496375660626, 0.0, 0.1521898252840332],[0.0, 0.13060867295881992, 0.0, 0.9914339990881572]]]

		self.listener = tf.TransformListener(True, rospy.Duration(10.0))
		self.arm_state = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state);
		
		rospy.loginfo("Waiting %s service...",  grasping_functions.graspingutils.get_ik_srv_name());
		rospy.wait_for_service( grasping_functions.graspingutils.get_ik_srv_name())
		rospy.loginfo("%s is ready.",  grasping_functions.graspingutils.get_ik_srv_name());

		rospy.loginfo("Waiting /get_db_grasps service...");
		rospy.wait_for_service('/get_db_grasps')
		self.client = rospy.ServiceProxy('/get_db_grasps', GetDBGrasps)
		rospy.loginfo("/get_feasible_grasps service is ready.");


	def get_feasible_grasps(self, request):
		x = time.time();
		rospy.loginfo("/get_feasible_grasps service has been called...");

		pregrasp_offsets = request.pregrasp_offsets
		if len(pregrasp_offsets) != 2:
			print "pregrasps_offsets value must be an array with length 2: [X,Z]"
			print "Using default values: [0.5, 0.0]"
			pregrasp_offsets = [0.0, 0.0];

		client_response = self.client(request.object_id)
		grasp_configuration = client_response.grasp_configuration;
		error_code = client_response.error_code.val

		#current_joint_configuration
		while self.arm_state.get_num_connections() == 0:
			time.sleep(0.3);
			continue;

		rotacion = grasping_functions.graspingutils.rotation_matrix(request.object_pose);


		resp = GetFeasibleGraspsResponse();
		resp.error_code.val = 0;
		resp.feasible_grasp_available = False;
		resp.grasp_configuration = []

		if error_code == GraspingErrorCodes.SUCCESS:

			for grasp_configuration in grasp_configuration:
				pre_trans = rotacion * grasping_functions.graspingutils.matrix_from_pose(grasp_configuration.pre_grasp.pose);
				grasp_trans = rotacion *  grasping_functions.graspingutils.matrix_from_pose(grasp_configuration.grasp.pose);

				pre = grasping_functions.graspingutils.pose_from_matrix(pre_trans);
				g = grasping_functions.graspingutils.pose_from_matrix(grasp_trans);
				surface_distance = g.pose.position.z - request.object_pose.position.z
				category = grasping_functions.graspingutils.get_grasp_category(pre.pose.position, g.pose.position);
				
				nvg = FeasibleGrasp(grasp_configuration.sdh_joint_values, "/sdh_palm_link", g, pre, surface_distance, category);
				fpos = self.get_finger_positions(category)
				pre.pose = grasping_functions.graspingutils.set_pregrasp_offsets(category, pre.pose, pregrasp_offsets);
			
				if (grasping_functions.graspingutils.valid_grasp(category)) and (not self.checkCollisions(fpos, nvg, request.object_pose)):
					class FeasibleGraspFound(Exception): pass
					try:
						for w in range(0, self.ik_loop_reply):
							(pgc, error) = grasping_functions.graspingutils.callIKSolver(self.cjc, pre);			
							if(error.val == error.SUCCESS):
								for k in range(0,self.ik_loop_reply):
									(gc, error) = grasping_functions.graspingutils.callIKSolver(pgc, g);
									if(error.val == error.SUCCESS):
										raise FeasibleGraspFound();
					except FeasibleGraspFound:
						resp.feasible_grasp_available = True;
						resp.grasp_configuration.append(nvg);
						resp.error_code.val = GraspingErrorCodes.SUCCESS

			#order grasps
			if len(resp.grasp_configuration) > 0:
				(resp.grasp_configuration).sort();
			else:
				resp.error_code.val = GraspingErrorCodes.GOAL_UNREACHABLE
		else:
			resp.error_code.val = error_code;


		rospy.loginfo(str(len(resp.grasp_configuration))+" valid grasps for this values.");	
		rospy.loginfo("/get_feasible_grasps call has finished with error_code: "+str(resp.error_code));
		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";
		return resp;
	 


	def __cmp__(self, other):
		if other.grasp.position.z > self.grasp.position.z:
			return 1
		elif other.grasp.position.z < self.grasp.position.z:
			return -1
		else:
			return 0


	def get_joint_state(self, msg):
		self.cjc = list(msg.desired.positions);


	def get_finger_positions(self, category):
		fing_pos = [];
		trans = (self.cylopen_position, self.spheropen_position)[category == "TOP"]

		fp1 = PoseStamped();
		fp2 = PoseStamped();
		fp3 = PoseStamped();

		for finger in range(0,len(trans)):
			fp1.pose.position.x = trans[finger][0][0]
			fp1.pose.position.y = trans[finger][0][1]
			fp1.pose.position.z = trans[finger][0][2]
	
			fp1.pose.orientation.x = trans[finger][1][0]
			fp1.pose.orientation.y = trans[finger][1][1]
			fp1.pose.orientation.z = trans[finger][1][2]
			fp1.pose.orientation.w = trans[finger][1][3]

		fp1.header.frame_id = "/sdh_finger_13_link"
		fp2.header.frame_id = "/sdh_finger_23_link"
		fp3.header.frame_id = "/sdh_thumb_3_link"
		fing_pos.append(fp1);
		fing_pos.append(fp2);
		fing_pos.append(fp3);

		return fing_pos;


	def transform_finger_positions(self, fpositions, g):
		response = [];

		for finger_pos in fpositions:
			ps = PoseStamped();
			ps.header.stamp = self.listener.getLatestCommonTime("/sdh_palm_link", finger_pos.header.frame_id);
			ps = self.listener.transformPose("/sdh_palm_link", finger_pos);

			#transform to object position
			matrix = grasping_functions.graspingutils.matrix_from_pose(g) * grasping_functions.graspingutils.matrix_from_pose(ps.pose);
			response.append(grasping_functions.graspingutils.pose_from_matrix(matrix));

		return response;


	def checkCollisions(self, fpositions, nvg, obj_pose):

		self.finger_correction = (0, -0.0685)[nvg.category == "TOP"];
		finger_pos = self.transform_finger_positions(fpositions, nvg.grasp.pose);

		for fp in finger_pos:
			if (obj_pose.position.z + 0.05)> (fp.pose.position.z + self.finger_correction):
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
