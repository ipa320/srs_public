#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.srv import SymbolGroundingPrimitiveBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *



def symbol_grounding_primitive_base_pose_client(obj_pose):

	rospy.wait_for_service('symbol_grounding_primitive_base_pose')

	symbol_grounding_primitive_base_pose = rospy.ServiceProxy('symbol_grounding_primitive_base_pose', SymbolGroundingPrimitiveBasePose)
	
	try:
		
		resp = symbol_grounding_primitive_base_pose(obj_pose)
		return resp

	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e


if __name__ == "__main__":
	
	
	
	obj_pose = Pose()

	obj_pose.position.x = 1.1
	obj_pose.position.y = 1.2
	obj_pose.position.z = 1.3

	obj_pose.orientation.x = 1.4
	obj_pose.orientation.y = 1.5
	obj_pose.orientation.z = 1.6
	obj_pose.orientation.w = 1.7

	print "Requesting primitive base pose."		
	
	pbp = symbol_grounding_primitive_base_pose_client(obj_pose)
	
	print pbp

	

