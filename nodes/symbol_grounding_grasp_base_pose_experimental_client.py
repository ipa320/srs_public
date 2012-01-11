#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import *
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import tf

def symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, parent_obj_geometry, furniture_geometry_list):


	rospy.wait_for_service('symbol_grounding_grasp_base_pose_experimental')
	symbol_grounding_grasp_base_pose_experimental = rospy.ServiceProxy('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental)

	try:
		resp1 = symbol_grounding_grasp_base_pose_experimental(target_obj_pose, parent_obj_geometry, furniture_geometry_list)
		return resp1
	
	except rospy.ServiceException, e:
		print "Service call failed: %s" %e


		
def getWorkspaceOnMap():
	#print 'test get all workspace (furnitures basically here) from map'
	try:
		requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
		resp2 = requestNewTask('ipa-kitchen-map', True)
		return resp2
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		
		



if __name__ == "__main__":



	target_obj_pose = Pose()

	target_obj_pose.position.x = -3.15
	target_obj_pose.position.y = 0.2
	target_obj_pose.position.z = 1.1
	target_obj_pose.orientation.x = 0
	target_obj_pose.orientation.y = 0
	target_obj_pose.orientation.z = 0
	target_obj_pose.orientation.w = 1


	workspace_info = getWorkspaceOnMap()	
	
	parent_obj_geometry = SRSSpatialInfo()
	
	parent_obj_geometry.pose.position.x = workspace_info.objectsInfo[0].pose.position.x
	parent_obj_geometry.pose.position.y = workspace_info.objectsInfo[0].pose.position.y
	parent_obj_geometry.pose.position.z = workspace_info.objectsInfo[0].pose.position.z
	parent_obj_geometry.pose.orientation.x = workspace_info.objectsInfo[0].pose.orientation.x
	parent_obj_geometry.pose.orientation.y = workspace_info.objectsInfo[0].pose.orientation.y
	parent_obj_geometry.pose.orientation.z = workspace_info.objectsInfo[0].pose.orientation.z
	parent_obj_geometry.pose.orientation.w = workspace_info.objectsInfo[0].pose.orientation.w
	parent_obj_geometry.l = workspace_info.objectsInfo[0].l
	parent_obj_geometry.w = workspace_info.objectsInfo[0].w
	parent_obj_geometry.h = workspace_info.objectsInfo[0].h

	furniture_geometry_list = list()
	furniture_geometry_list = workspace_info.objectsInfo


	print "Requesting reachability and grasp base pose."
	result = symbol_grounding_grasp_base_pose_experimental_client(target_obj_pose, parent_obj_geometry, furniture_geometry_list)
	print result

		

	

