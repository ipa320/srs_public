#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.srv import SymbolGroundingPrimitiveBasePose



def symbol_grounding_primitive_base_pose_client(obj_x, obj_y, obj_th):

	rospy.wait_for_service('symbol_grounding_primitive_base_pose')

	symbol_grounding_primitive_base_pose = rospy.ServiceProxy('symbol_grounding_primitive_base_pose', SymbolGroundingPrimitiveBasePose)
	
	try:
		
		resp = list()
		resp.append(symbol_grounding_primitive_base_pose(obj_x, obj_y, obj_th))
		return resp
	
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e


if __name__ == "__main__":
	
	obj_x = 1.1
	obj_y = 1.2
	obj_th = 1.3

	print "Requesting primitive base pose."		
	
	pbp = symbol_grounding_primitive_base_pose_client(obj_x, obj_y, obj_th)
	
	print pbp

	

