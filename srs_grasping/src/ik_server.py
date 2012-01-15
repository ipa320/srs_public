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

class ik_action_server():

	def __init__(self):

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


    	def matrix_from_graspPose(self,gp):
		q = []
		q.append(gp.pose.orientation.x)
		q.append(gp.pose.orientation.y)
		q.append(gp.pose.orientation.z)
		q.append(gp.pose.orientation.w)
		e = euler_from_quaternion(q, axes='sxyz')

		m = euler_matrix(e[0],e[1],e[2] ,axes='sxyz')
		m[0][3] = gp.pose.position.x
		m[1][3] = gp.pose.position.y
		m[2][3] = gp.pose.position.z

		m = matrix([[m[0][0], m[0][1], m[0][2], m[0][3]], [m[1][0], m[1][1], m[1][2], m[1][3]], [m[2][0], m[2][1], m[2][2], m[2][3]], [m[3][0], m[3][1], m[3][2], m[3][3]]])
		return m


    	def get_joint_state(self, msg):
		global current_joint_configuration
		current_joint_configuration = list(msg.desired.positions)
		rospy.spin()


	def execute_cb(self, server_goal):
		x = time.time()
		server_result = IKActionResult().result


		#current_joint_configuration
		sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.get_joint_state)
		while sub.get_num_connections() == 0:
			time.sleep(0.3)
			continue


		sol = False
		for w in range(0,10):
			(pre_grasp_conf, error_code) = self.callIKSolver(current_joint_configuration, server_goal.pre_pose)		
			if(error_code.val == error_code.SUCCESS):
				for k in range(0,10):
					(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, server_goal.g_pose)
					if(error_code.val == error_code.SUCCESS):		
						print str(i)+": IK solution found"
						sol = True
						break
				if sol:
					server_result.pre_position = pre_grasp_conf
					server_result.g_position = grasp_conf
					self.ik_action_server.set_succeeded(server_result)
				else:

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
