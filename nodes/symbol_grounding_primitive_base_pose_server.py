#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingPrimitiveBasePose
import rospy
import math



def handle_symbol_grounding_primitive_base_pose(req):

	pbp = list()
	pbp.append(req.obj_x + 0.8*math.cos(req.obj_th) + 0.1*math.sin(req.obj_th))
	pbp.append(req.obj_y + 0.1*math.cos(req.obj_th) - 0.8*math.sin(req.obj_th))
	pbp.append(req.obj_th)
	pbp = [pbp]

	return pbp



def symbol_grounding_primitive_base_pose_server():

	rospy.init_node('symbol_grounding_primitive_base_pose_server')
	s = rospy.Service('symbol_grounding_primitive_base_pose', SymbolGroundingPrimitiveBasePose, handle_symbol_grounding_primitive_base_pose)
	print "Ready to receive requests."
	rospy.spin()


if __name__ == "__main__":
	symbol_grounding_primitive_base_pose_server()
