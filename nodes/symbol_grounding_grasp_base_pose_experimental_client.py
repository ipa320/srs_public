#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePoseExperimental
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy


def symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, rb_pose, parent_obj_geometry, furnitures_geometry):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose_experimental')
	symbol_grounding_grasp_base_pose_experimental = rospy.ServiceProxy('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental)

	try:
		resp = symbol_grounding_grasp_base_pose_experimental(target_obj_pose, rb_pose, parent_obj_geometry, furnitures_geometry)
		return resp
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e
		
		



if __name__ == "__main__":
	

	target_obj_pose = Pose()

	target_obj_pose.position.x = 1.1
	target_obj_pose.position.y = 1.2
	target_obj_pose.position.z = 1.3
	target_obj_pose.orientation.x = 1.4
	target_obj_pose.orientation.y = 1.5
	target_obj_pose.orientation.z = 1.6
	target_obj_pose.orientation.w = 1.7

	rb_pose = Pose2D()
	
	rb_pose.x = 1.8
	rb_pose.y = 1.5
	rb_pose.theta = 0.3

	parent_obj_geometry = SRSFurnitureGeometry()
	
	parent_obj_geometry.pose.x = 0.8
	parent_obj_geometry.pose.y = 1.2
	parent_obj_geometry.pose.theta = 0.3
	parent_obj_geometry.l = 1.5
	parent_obj_geometry.w = 0.8
	parent_obj_geometry.h = 1.0

	furnitures_geometry = [0] * 2
	furnitures_geometry[0] = SRSFurnitureGeometry()
	furnitures_geometry[1] = SRSFurnitureGeometry()

	furnitures_geometry[0].pose.x = 1.0
	furnitures_geometry[0].pose.y = 1.2
	furnitures_geometry[0].pose.theta = 0.5
	furnitures_geometry[0].l = 0.8
	furnitures_geometry[0].w = 0.8
	furnitures_geometry[0].h = 1.0

	furnitures_geometry[1].pose.x = 1.8
	furnitures_geometry[1].pose.y = 1.7
	furnitures_geometry[1].pose.theta = 0.2
	furnitures_geometry[1].l = 0.8
	furnitures_geometry[1].w = 0.8
	furnitures_geometry[1].h = 1.0


	print "Requesting reachability and grasp base pose."
	result = symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, rb_pose, parent_obj_geometry, furnitures_geometry)
	print result

		

	

