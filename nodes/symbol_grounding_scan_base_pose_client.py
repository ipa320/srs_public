#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose



def symbol_grounding_scan_base_pose_client(table_x, table_y, table_th, table_length, table_width):

	rospy.wait_for_service('symbol_grounding_scan_base_pose')
	
	symbol_grounding_scan_base_pose = rospy.ServiceProxy('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose)
	
	try:

		resp = list()
		resp.append(symbol_grounding_scan_base_pose(table_x, table_y, table_th, table_length, table_width))
		return resp
	
	except rospy.ServiceException, e:
		
		print "Service call failed: %s" %e


if __name__ == "__main__":
	
	table_x = -0.5
	table_y = -0.3 
	table_th = 0.15
	table_length = 2.0
	table_width = 1.6

	print "Requesting scan base pose."
	
	sbp = symbol_grounding_scan_base_pose_client(table_x, table_y, table_th, table_length, table_width)
	
	print sbp
		


