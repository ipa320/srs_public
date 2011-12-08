#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePose


def symbol_grounding_grasp_base_pose_client(grasp, rb_x, rb_y, rb_th, obj_x, obj_y, obj_th):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose')
	symbol_grounding_grasp_base_pose = rospy.ServiceProxy('symbol_grounding_grasp_base_pose', SymbolGroundingGraspBasePose)

	try:
		resp = list()
		resp.append(symbol_grounding_grasp_base_pose(grasp, rb_x, rb_y, rb_th, obj_x, obj_y, obj_th))
		return resp
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
		
		



if __name__ == "__main__":
	
	
	grasp = 1
	rb_x = -0.68
	rb_y = 0.43
	rb_th = 0.15
	obj_x = -1.5
	obj_y = 0.4
	obj_th = 0.14


	print "Requesting reachability and grasp base pose."
	gbp = symbol_grounding_grasp_base_pose_client(grasp, rb_x, rb_y, rb_th, obj_x, obj_y, obj_th)
	print gbp

		

	

