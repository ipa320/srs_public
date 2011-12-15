#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy


def symbol_grounding_grasp_base_pose_client(grasp, obj_pose, rb_pose):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose')
	symbol_grounding_grasp_base_pose = rospy.ServiceProxy('symbol_grounding_grasp_base_pose', SymbolGroundingGraspBasePose)

	try:
		resp = symbol_grounding_grasp_base_pose(grasp, obj_pose, rb_pose)
		return resp
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
		
		



if __name__ == "__main__":
	
	grasp = 1
	
	obj_pose = Pose()
	
	obj_pose.position.x = 1.1
	obj_pose.position.y = 1.2
	obj_pose.position.z = 1.3
	
	obj_pose.orientation.x = 1.4
	obj_pose.orientation.y = 1.5
	obj_pose.orientation.z = 1.6
	obj_pose.orientation.w = 1.7

	rb_pose = Pose2D()
	
	rb_pose.x = 0.95
	rb_pose.y = 2
	rb_pose.theta = 1.56


	print "Requesting reachability and grasp base pose."
	gbp = symbol_grounding_grasp_base_pose_client(grasp, obj_pose, rb_pose)
	print gbp

		

	

