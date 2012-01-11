#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import time
import rospy
import actionlib
import os

import grasping_functions
import generator

from srs_grasping.msg import *
from srs_object_database.srv import *


from srs_grasping.msg import *
from kinematics_msgs.srv import *
from pr2_controllers_msgs.msg import *


class ik_action_server():

	def __init__(self):

		self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)

		self.ns_global_prefix = "/ik_server"
		self.ik_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, IKAction, self.execute_cb, True)
		self.ik_action_server.start()


	def callIKSolver(self, current_pose, goal_pose):

		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_palm_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

    	def get_joint_state(self, msg):

		global current_joint_configuration
		current_joint_configuration = list(msg.desired.positions)
		rospy.spin()


	def execute_cb(self, server_goal):
		x = time.time()
		server_result = IKActionResult().result

		sol = False
		sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
		while sub.get_num_connections() == 0:
			time.sleep(0.3)
			continue

		for w in range(0,10):
			(pre_grasp_conf, error_code) = self.callIKSolver(current_joint_configuration, server_goal.pre_pose)		
			if(error_code.val == error_code.SUCCESS):
				for k in range(0,10):
					(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, server_goal.g_pose)
					if(error_code.val == error_code.SUCCESS):		
						sol = True
						break
				if sol:
					break
		if sol:
			print "IK solution found"
			server_result.pre_position = pre_grasp_conf
			server_result.g_position = grasp_conf
			self.ik_action_server.set_aborted(server_result)
		else:
			print "IK solution not found"
			server_result.pre_position = []
			server_result.g_position = []
			self.ik_action_server.set_aborted(server_result)


		print "Time employed: ",time.time()-x
		print "-------------------------------"


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('ik_server')

	SCRIPT = ik_action_server()
	rospy.loginfo("/ik_server is running")
	rospy.spin()
