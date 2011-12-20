#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion


def symbol_grounding_scan_base_pose_client(furniture_geometry):

	rospy.wait_for_service('symbol_grounding_scan_base_pose')
	
	symbol_grounding_scan_base_pose = rospy.ServiceProxy('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose)
	
	try:
		resp = list()
		resp.append(symbol_grounding_scan_base_pose(furniture_geometry))
		return resp
	
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e


if __name__ == "__main__":
	
	furniture_geometry = SRSFurnitureGeometry()
	furniture_geometry.pose.x = -3.2
	furniture_geometry.pose.y = -0.54
	furniture_geometry.pose.theta = 0
	furniture_geometry.l = 0.6
	furniture_geometry.w = 0.6
	furniture_geometry.h = 0.85

	print "Requesting scan base pose."
	
	sbps = symbol_grounding_scan_base_pose_client(furniture_geometry)
	
	print sbps
		


