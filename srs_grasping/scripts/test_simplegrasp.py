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
#   ROS stack name: srs_public
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
#   Implements a simple grasp test.
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
import tf
import sys
import grasping_functions

from simple_script_server import *
from pr2_controllers_msgs.msg import *
from geometry_msgs.msg import *
from srs_grasping.srv import *
from cob_object_detection_msgs.srv import *

#from numpy import matrix


class GraspScript(script):
		
	def __init__(self):
		
		self.sss = simple_script_server()
		self.detection_service = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
		self.arm_state = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))


		# initialize components (not needed for simulation)
		"""
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("base")
		"""

		# move to initial positions
		handle_arm = self.sss.move("arm","folded",False)
		handle_torso = self.sss.move("torso","home",False)
		handle_sdh = self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle_arm.wait()
		handle_torso.wait()
		handle_sdh.wait()
		
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		#self.sss.wait_for_input()
		

	def run(self, object_id):

		# prepare for grasping
		#self.sss.move("base","kitchen")
		self.sss.move("arm","look_at_table")
		self.sss.move("sdh","cylopen")
		rospy.sleep(2);

		#current_joint_configuration
		while self.arm_state.get_num_connections() == 0:
			time.sleep(0.3)
			continue

		#Detection
		req = DetectObjectsRequest()
		req.object_name.data = grasping_functions.databaseutils.get_object_name(object_id);
		res = self.detection_service(req)
		
		print "---------------------------------------------------------"
		for i in range(0,len(res.object_list.detections)):
			print str(i)+": "+res.object_list.detections[i].label
		print "---------------------------------------------------------"

		index = -1;
		while (index < 0):
			index = 0#int(raw_input("Select object to grasp: "))
		
		obj = res.object_list.detections[index].pose
		obj.header.stamp = self.listener.getLatestCommonTime("/base_link", obj.header.frame_id)
		obj = self.listener.transformPose("/base_link", obj)
		print "obj_pose:\n",obj.pose



		print "Calling get_feasible_grasps service..."
		get_grasps_from_position = rospy.ServiceProxy('get_feasible_grasps', GetFeasibleGrasps)
		req = GetFeasibleGraspsRequest(object_id, obj.pose, [0.0, 0.0])	
		grasp_configuration = (get_grasps_from_position(req)).grasp_configuration
		print "get_feasible_grasps service has finished."


		for i in range(0,len(grasp_configuration)):

			pre = PoseStamped()
			pre.header.stamp = rospy.Time.now()
			pre.header.frame_id = "/base_link"
			pre.pose.position.x = grasp_configuration[i].pre_grasp.pose.position.x
			pre.pose.position.y = grasp_configuration[i].pre_grasp.pose.position.y
			pre.pose.position.z = grasp_configuration[i].pre_grasp.pose.position.z
			pre.pose.orientation.x = grasp_configuration[i].pre_grasp.pose.orientation.x
			pre.pose.orientation.y = grasp_configuration[i].pre_grasp.pose.orientation.y
			pre.pose.orientation.z = grasp_configuration[i].pre_grasp.pose.orientation.z
			pre.pose.orientation.w = grasp_configuration[i].pre_grasp.pose.orientation.w

			g = PoseStamped()
			g.header.stamp = rospy.Time.now()
			g.header.frame_id = "/base_link"
			g.pose.position.x = grasp_configuration[i].grasp.pose.position.x
			g.pose.position.y = grasp_configuration[i].grasp.pose.position.y
			g.pose.position.z = grasp_configuration[i].grasp.pose.position.z
			g.pose.orientation.x = grasp_configuration[i].grasp.pose.orientation.x
			g.pose.orientation.y = grasp_configuration[i].grasp.pose.orientation.y
			g.pose.orientation.z = grasp_configuration[i].grasp.pose.orientation.z
			g.pose.orientation.w = grasp_configuration[i].grasp.pose.orientation.w
	
			sol = False
			for w in range(0,5):
				(pre_grasp_conf, error_code) = grasping_functions.graspingutils.callIKSolver(current_joint_configuration, pre);
				if(error_code.val == error_code.SUCCESS):
					for k in range(0,5):
						(grasp_conf, error_code) = grasping_functions.graspingutils.callIKSolver(pre_grasp_conf, g);
						if(error_code.val == error_code.SUCCESS):		
							print str(i)+": IK solution found"
							sol = True
							break
					if sol:
						break


			if sol:
				print "category:",grasp_configuration[i].category
				print "grasp position:\n",g.pose.position
				res = raw_input("Execute this grasp? (y/n): ")

				if res != "y":	
					continue
				else:
					#grasping_functions.openraveutils.grasp_view(object_id, grasp_configuration[i], obj.pose)
					# execute grasp
					handle_say = self.sss.say(["I am grasping the object now."], False)
					handle_arm = self.sss.move("arm", [pre_grasp_conf], False)
					self.sss.move("sdh", "cylopen")
					handle_say.wait()
					handle_arm.wait()

					raw_input("Grasp...")
					handle_arm2 = self.sss.move("arm", [grasp_conf], False)
					handle_arm2.wait()

					raw_input("Catch the object")
					handle_sdh = self.sss.move("sdh", [list(grasp_configuration[i].sdh_joint_values)], False)
					handle_sdh.wait()
					rospy.sleep(4);

		
					# place obj on tray
					handle01 = self.sss.move("arm","grasp-to-tray",False)
					self.sss.move("tray","up")
					handle01.wait()

					self.sss.move("arm","tray")
					self.sss.move("sdh","cylopen")
					handle01 = self.sss.move("arm","tray-to-folded",False)
					self.sss.sleep(4)
					self.sss.move("sdh","cylclosed",False)
					handle01.wait()

					# deliver cup to order position
					#self.sss.move("base","order")
					#self.sss.say("Here's your drink.")
					#self.sss.move("torso","nod")

					res = grasping_functions.graspingutils.sdh_tactil_sensor_result()
					if not res:
						val = list(grasp_configuration[i].sdh_joint_values)
						val[1] += 0.07
						val[3] += 0.07
						val[5] += 0.07
						handle_sdh = self.sss.move("sdh", [val], False)
						handle_sdh.wait()
						rospy.sleep(4);
						return 0;
					
					return 0;
		return -1


    	def get_joint_state(self, msg):
		global current_joint_configuration
		current_joint_configuration = list(msg.desired.positions)
		rospy.spin()


if __name__ == "__main__":

    	rospy.init_node('grasp_test')
	print "---------------------------------------------------------------------------------------"
	print "usage:\t\trosrun srs_grasping test_graspMilk.py [object_id]\ndefault:\tobject_id: 9 (Milkbox)"
	print "---------------------------------------------------------------------------------------"
	s = GraspScript()

	if len(sys.argv) == 1:
		s.run(9)
	else:
    		s.run(int(sys.argv[1]))

