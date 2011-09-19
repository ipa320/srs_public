#!/usr/bin/python

import time
import roslib
roslib.load_manifest('srs_grasping')
import rospy
import actionlib

import grasping_functions
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from srs_grasping.msg import *
from srs_msgs.msg import *


class grasp_action_server():

	def __init__(self):
		self.package_path = roslib.packages.get_pkg_dir('srs_grasping')
		self.ns_global_prefix = "/grasp_server"
		self.grasp_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, GraspAction, self.execute_cb, True)
		self.grasp_action_server.start()
	
	def execute_cb(self, server_goal):	
		x = time.time()
		server_result = GraspActionResult().result


		try:
			# SE DEBE IMPLEMENTAR UNA FUNCION QUE DADO UN FLOAT object_id SEPA A QUE OBJETO SE CORRESPONDE
			# DE MOMENTO UTILIZO UN IF CHAPUCERO
			if server_goal.object_id == 0:
				object_name = "milk_box"
			else:
				object_name = "OTHER"
				

			file_name = self.package_path+"/DB/"+object_name+"_all_grasps.xml"

			GRASPS = grasping_functions.getGrasps(file_name, msg=True)
			rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.")		

			server_result.grasp_configuration = GRASPS
			self.grasp_action_server.set_succeeded(server_result)

		except:
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
