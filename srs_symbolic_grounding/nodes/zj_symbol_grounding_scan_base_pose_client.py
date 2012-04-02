#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy



def symbol_grounding_scan_base_pose_client(parent_obj_geometry, furniture_geometry_list):

	rospy.wait_for_service('symbol_grounding_scan_base_pose')
	
	symbol_grounding_scan_base_pose = rospy.ServiceProxy('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose)
	
	try:
		resp = list()
		resp.append(symbol_grounding_scan_base_pose(parent_obj_geometry, furniture_geometry_list))
		return resp
	
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e


if __name__ == "__main__":
	
	parent_obj_geometry = SRSFurnitureGeometry()
	
	parent_obj_geometry.pose.position.x = -3.2
	parent_obj_geometry.pose.position.y = 0.16
	parent_obj_geometry.pose.position.z = 0.4
	parent_obj_geometry.pose.orientation.x = 0
	parent_obj_geometry.pose.orientation.y = 0
	parent_obj_geometry.pose.orientation.z = 0
	parent_obj_geometry.pose.orientation.w = 1
	parent_obj_geometry.l = 0.6
	parent_obj_geometry.w = 0.6
	parent_obj_geometry.h = 1.3


	furniture_geometry_1 = SRSFurnitureGeometry()
	furniture_geometry_2 = SRSFurnitureGeometry()
	furniture_geometry_3 = SRSFurnitureGeometry()
	furniture_geometry_4 = SRSFurnitureGeometry()

	furniture_geometry_1.pose.position.x = -3.2
	furniture_geometry_1.pose.position.y = -0.54
	furniture_geometry_1.pose.orientation.x = 0
	furniture_geometry_1.pose.orientation.y = 0
	furniture_geometry_1.pose.orientation.z = 0
	furniture_geometry_1.pose.orientation.w = 1
	furniture_geometry_1.l = 0.6
	furniture_geometry_1.w = 0.6
	furniture_geometry_1.h = 1.4

	furniture_geometry_2.pose.position.x = -3.2
	furniture_geometry_2.pose.position.y = 1.36
	furniture_geometry_2.pose.orientation.x = 0
	furniture_geometry_2.pose.orientation.y = 0
	furniture_geometry_2.pose.orientation.z = 0
	furniture_geometry_2.pose.orientation.w = 1
	furniture_geometry_2.l = 0.6
	furniture_geometry_2.w = 0.6
	furniture_geometry_2.h = 1.4


	furniture_geometry_3.pose.position.x = -3.2
	furniture_geometry_3.pose.position.y = 0.76
	furniture_geometry_3.pose.orientation.x = 0
	furniture_geometry_3.pose.orientation.y = 0
	furniture_geometry_3.pose.orientation.z = 0
	furniture_geometry_3.pose.orientation.w = 1
	furniture_geometry_3.l = 0.6
	furniture_geometry_3.w = 0.6
	furniture_geometry_3.h = 1.4

	furniture_geometry_4.pose.position.x = -3.2
	furniture_geometry_4.pose.position.y = -1.04
	furniture_geometry_4.pose.orientation.x = 0
	furniture_geometry_4.pose.orientation.y = 0
	furniture_geometry_4.pose.orientation.z = 0
	furniture_geometry_4.pose.orientation.w = 1
	furniture_geometry_4.l = 0.6
	furniture_geometry_4.w = 0.6
	furniture_geometry_4.h = 1.4

	furniture_geometry_list = [furniture_geometry_1, furniture_geometry_2, furniture_geometry_3, furniture_geometry_4]

	print "Requesting scan base pose."
	
	result = symbol_grounding_scan_base_pose_client(parent_obj_geometry, furniture_geometry_list)
	
	print result
