#!/usr/bin/python

import roslib
roslib.load_manifest('cob_generic_states')

import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

import tf
from kinematics_msgs.srv import *


## Grasp side state
#
# This state will grasp an object with a side grasp
class grasp_side(smach.State):

	def __init__(self, max_retries = 1):
		smach.State.__init__(
			self,
			outcomes=['succeeded', 'retry', 'no_more_retries', 'failed'],
			input_keys=['object'])
		
		self.max_retries = max_retries
		self.retries = 0
		self.iks = rospy.ServiceProxy('/arm_controller/get_ik', GetPositionIK)
		self.listener = tf.TransformListener()
		#self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

	def callIKSolver(self, current_pose, goal_pose):
		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_grasp_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)

	def execute(self, userdata):
		# check if maximum retries reached
		if self.retries > self.max_retries:
			self.retries = 0
			return 'no_more_retries'
		"""
		# make arm soft
		try:
			self.stiffness([300,300,300,300,300,300,300])
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.retries = 0
			return 'failed'
		"""

		# transform object_pose into base_link
		object_pose_in = userdata.object.pose
		#object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
		object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
	
		[new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.552, -0.042, 2.481) # rpy 
		object_pose_bl.pose.orientation.x = new_x
		object_pose_bl.pose.orientation.y = new_y
		object_pose_bl.pose.orientation.z = new_z
		object_pose_bl.pose.orientation.w = new_w

		# HACK: this is calibration between camera and hand
		object_pose_bl.pose.position.x = object_pose_bl.pose.position.x - 0.08
		#object_pose_bl.pose.position.y = object_pose_bl.pose.position.y - 0.05
		object_pose_bl.pose.position.z = object_pose_bl.pose.position.z + 0.03
		
		# calculate pre and post grasp positions
		pre_grasp_bl = PoseStamped()
		post_grasp_bl = PoseStamped()
		pre_grasp_bl = copy.deepcopy(object_pose_bl)
		post_grasp_bl = copy.deepcopy(object_pose_bl)

		pre_grasp_bl.pose.position.x = pre_grasp_bl.pose.position.x + 0.05 # x offset for pre grasp position
		pre_grasp_bl.pose.position.y = pre_grasp_bl.pose.position.y + 0.05 # y offset for pre grasp position
		post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
		post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15 # z offset for post grasp position
		
		# calculate ik solutions for pre grasp configuration
		arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp_top")
		(pre_grasp_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_grasp_bl)		
		if(error_code.val != error_code.SUCCESS):
			print "Ik pre_grasp Failed"
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for grasp configuration
		(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, object_pose_bl)
		if(error_code.val != error_code.SUCCESS):
			print "Ik grasp Failed"
			self.retries += 1
			return 'retry'
		
		# calculate ik solutions for pre grasp configuration
		(post_grasp_conf, error_code) = self.callIKSolver(grasp_conf, post_grasp_bl)
		if(error_code.val != error_code.SUCCESS):
			print "Ik post_grasp Failed"
			self.retries += 1
			return 'retry'	

		# execute grasp
		sss.say(["I am grasping the " + userdata.object.label + " now."],False)
		sss.move("torso","home")
		handle_arm = sss.move("arm", [pre_grasp_conf , grasp_conf],False)
		sss.move("sdh", "cylopen")
		handle_arm.wait()
		sss.move("sdh", "cylclosed")
	
		# move object to frontside and put object on tray
		sss.move("head","front",False)
		handle_arm = sss.move("arm", [post_grasp_conf, "intermediateback","intermediatefront","intermediatefront", "overtray"],False)
		sss.sleep(1)
		handle_tray = sss.move("tray","up",False)
		handle_arm.wait()
		handle_tray.wait()
		sss.move("sdh","cylopen")
		
		# move arm back to folded
		handle_arm = sss.move("arm","tray-to-folded",False)
		sss.sleep(2)
		sss.move("sdh","home",False)
		
		return 'succeeded'
