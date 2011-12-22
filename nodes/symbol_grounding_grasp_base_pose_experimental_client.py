#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePoseExperimental
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy


def symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, rb_pose, parent_obj_geometry, furniture_geometry_list):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose_experimental')
	symbol_grounding_grasp_base_pose_experimental = rospy.ServiceProxy('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental)

	try:
		resp = symbol_grounding_grasp_base_pose_experimental(target_obj_pose, rb_pose, parent_obj_geometry, furniture_geometry_list)
		return resp
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
		
		



if __name__ == "__main__":
	

	target_obj_pose = Pose()

	target_obj_pose.position.x = -0.7
	target_obj_pose.position.y = -0.1
	target_obj_pose.position.z = 1.1
	target_obj_pose.orientation.x = 1.4
	target_obj_pose.orientation.y = 1.5
	target_obj_pose.orientation.z = 1.6
	target_obj_pose.orientation.w = 1.7

	rb_pose = Pose2D()
	
	rb_pose.x = -0.55
	rb_pose.y = -0.9
	rb_pose.theta = 4.7

	parent_obj_geometry = SRSFurnitureGeometry()
	
	parent_obj_geometry.pose.position.x = -0.5
	parent_obj_geometry.pose.position.y = 0.2
	parent_obj_geometry.pose.position.z = 0.5
	parent_obj_geometry.pose.orientation.x = 1.4
	parent_obj_geometry.pose.orientation.y = 1.5
	parent_obj_geometry.pose.orientation.z = 1.6
	parent_obj_geometry.pose.orientation.w = 1.7
	parent_obj_geometry.l = 1.5
	parent_obj_geometry.w = 0.8
	parent_obj_geometry.h = 1.0


	furniture_geometry_1 = SRSFurnitureGeometry()
	furniture_geometry_2 = SRSFurnitureGeometry()

	furniture_geometry_1.pose.position.x = 1.6
	furniture_geometry_1.pose.position.y = 1.7
	furniture_geometry_1.pose.orientation.x = 1.4
	furniture_geometry_1.pose.orientation.y = 1.5
	furniture_geometry_1.pose.orientation.z = 1.6
	furniture_geometry_1.pose.orientation.w = 1.7
	furniture_geometry_1.l = 0.8
	furniture_geometry_1.w = 0.8
	furniture_geometry_1.h = 1.0

	furniture_geometry_2.pose.position.x = 1.8
	furniture_geometry_2.pose.position.y = 1.9
	furniture_geometry_2.pose.orientation.x = 1.4
	furniture_geometry_2.pose.orientation.y = 1.5
	furniture_geometry_2.pose.orientation.z = 1.6
	furniture_geometry_2.pose.orientation.w = 1.7
	furniture_geometry_2.l = 0.8
	furniture_geometry_2.w = 0.8
	furniture_geometry_2.h = 1.0

	furniture_geometry_list = [furniture_geometry_1, furniture_geometry_2]


	print "Requesting reachability and grasp base pose."
	result = symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, rb_pose, parent_obj_geometry, furniture_geometry_list)
	print result

		

	

