#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import actionlib
import sys, os
import openravepy

from srs_grasping.msg import *
from tf.transformations import *


import grasping_functions
from srs_object_database.msg import *
from srs_object_database.srv import *
from tf.transformations import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from srs_grasping.msg import *
from srs_msgs.msg import *
from pr2_controllers_msgs.msg import *
from kinematics_msgs.srv import *
import grasping_functions
from numpy import matrix

##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):
		print "iniciando prueba"
		
	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	
		#TODO: Detect an object and transform its pose into base_link coordinates system.
		pre_grasp_pose = PoseStamped()

		pre_grasp_pose.header.stamp = rospy.Time.now()
		pre_grasp_pose.header.frame_id = "/base_link"
		pre_grasp_pose.pose.position.x = -0.25
		pre_grasp_pose.pose.position.y = 0.05
		pre_grasp_pose.pose.position.z = 0.6
		pre_grasp_pose.pose.orientation.x = 0
		pre_grasp_pose.pose.orientation.y = 0
		pre_grasp_pose.pose.orientation.z = 0
		pre_grasp_pose.pose.orientation.w = -1
		
		prueba = PoseStamped()
		prueba.header.stamp = rospy.Time.now()
		prueba.header.frame_id = "/base_link"
		prueba.pose.position.x = 7
		prueba.pose.position.y = 11
		prueba.pose.position.z = 13
		prueba.pose.orientation.x = 0
		prueba.pose.orientation.y = 0
		prueba.pose.orientation.z = 1
		prueba.pose.orientation.w = 0
		
		matrix1 = self.matrix_from_graspPose(pre_grasp_pose)
		matrix2 = self.matrix_from_graspPose(prueba)
		transformation = matrix2*matrix1.I

		

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

##########################################################################
if __name__ == "__main__":################################################
##########################################################################
    	rospy.init_node('prueba')
	s = SCRIPT()
    	s.run()
