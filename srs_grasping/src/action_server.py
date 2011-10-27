#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import time
import rospy
import actionlib

from srs_grasping.msg import *
import grasping_functions

class grasp_action_server():

	def __init__(self):
		self.package_path = roslib.packages.get_pkg_dir('srs_grasping')
		self.ns_global_prefix = "/grasp_server"
		self.grasp_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, GraspAction, self.execute_cb, True)
		self.grasp_action_server.start()
	
	def execute_cb(self, server_goal):	
		x = time.time()
		server_result = GraspActionResult().result


		#SE DEBE IMPLEMENTAR UNA FUNCION QUE DADO UN INR object_id SEPA A QUE OBJETO SE CORRESPONDE
		object_name = grasping_functions.getObjectName(server_goal.object_id)

		file_name = self.package_path+"/DB/"+object_name+"_all_grasps.xml"

		grasps = grasping_functions.getGrasps(file_name, msg=True)
		GRASPS = grasping_functions.getGraspsByAxis(grasps, server_goal.pose_id)

		rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.")		

		if len(GRASPS)>0:
			server_result.grasp_configuration = GRASPS
			self.grasp_action_server.set_succeeded(server_result)
		else:
			rospy.logerr("No grasping configurations for this object.")
			server_result.grasp_configuration = []
			self.grasp_action_server.set_aborted(server_result)


		print "Time employed: ",time.time()-x
		print "-----"

## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('get_grasps_action_server')
	SCRIPT = grasp_action_server()
	rospy.loginfo("/grasp_server is running")
	rospy.spin()
