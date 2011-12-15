#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingPrimitiveBasePose
from srs_symbolic_grounding.msg import *
import rospy
import math

import tf
from tf.transformations import euler_from_quaternion


def handle_symbol_grounding_primitive_base_pose(req):

	obj_x = req.obj_pose.position.x

	obj_y = req.obj_pose.position.y

	obj_rpy = tf.transformations.euler_from_quaternion([req.obj_pose.orientation.x, req.obj_pose.orientation.y, req.obj_pose.orientation.z, req.obj_pose.orientation.w])

	obj_th = obj_rpy[2]
	
	pbp = Pose2D()
	pbp.x = obj_x + 0.8*math.cos(obj_th) - 0.1*math.sin(obj_th)
	pbp.y = obj_y + 0.1*math.cos(obj_th) + 0.8*math.sin(obj_th)
	pbp.theta = obj_th

	return pbp



def symbol_grounding_primitive_base_pose_server():

	rospy.init_node('symbol_grounding_primitive_base_pose_server')
	s = rospy.Service('symbol_grounding_primitive_base_pose', SymbolGroundingPrimitiveBasePose, handle_symbol_grounding_primitive_base_pose)
	print "Ready to receive requests."
	rospy.spin()


if __name__ == "__main__":
	symbol_grounding_primitive_base_pose_server()
