#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import time
import rospy
import actionlib
import os

import grasping_functions
import generator

from srs_grasping.msg import *
from srs_object_database.srv import *

class grasp_action_server():

	def __init__(self):

		self.ns_global_prefix = "/grasp_server"
		self.grasp_action_server = actionlib.SimpleActionServer(self.ns_global_prefix, GraspAction, self.execute_cb, True)
		self.grasp_action_server.start()


	
	def execute_cb(self, server_goal):
		x = time.time()
		server_result = GraspActionResult().result

		if server_goal.pose_id != "X" and server_goal.pose_id != "-X" and server_goal.pose_id != "Y" and server_goal.pose_id != "-Y" and server_goal.pose_id != "Z" and server_goal.pose_id != "-Z" and server_goal.pose_id != "":
			rospy.logerr("Invalid pose_id.")
			server_result.grasp_configuration = []
			self.grasp_action_server.set_aborted(server_result)
		else:

			rospy.loginfo("Waiting for /get_model_grasp service...")
			rospy.wait_for_service('/get_model_grasp')
			rospy.loginfo("/get_model_grasp service found!")

			get_grasp = rospy.ServiceProxy('/get_model_grasp', GetGrasp)

			try:

				resp = get_grasp(model_ids=[server_goal.object_id])

				if len(resp.msg) == 0:
					rospy.loginfo("There is no pre-generated info for this object. It will be generate.")
					s = generator.SCRIPT()
					ret = s.run(server_goal.object_id)

					if ret == -1:
						rospy.logerr("No grasping configurations for this object.")
						server_result.grasp_configuration = []
						self.grasp_action_server.set_aborted(server_result)
					else:
						resp = get_grasp(model_ids=[server_goal.object_id])

						try:
							grasp_file = "/tmp/grasp.xml"
							f = open(grasp_file, 'w')
							f.write(resp.msg[0].bs)
							f.close()

							grasps = grasping_functions.getGrasps(grasp_file, all_grasps=True, msg=True)
							GRASPS = grasping_functions.getGraspsByAxis(grasps, server_goal.pose_id)
							os.remove(grasp_file)

							rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.")		

							if len(GRASPS)>0:
								server_result.grasp_configuration = GRASPS
								self.grasp_action_server.set_succeeded(server_result)
							else:
								rospy.logerr("No grasping configurations for this object.")
								server_result.grasp_configuration = []
								self.grasp_action_server.set_aborted(server_result)
			


						except rospy.ServiceException, e:
							rospy.logerr("Service did not process request: %s", str(e))
							rospy.logerr("No grasping configurations for this object.")
							server_result.grasp_configuration = []
							self.grasp_action_server.set_aborted(server_result)



				else:
					resp = get_grasp(model_ids=[server_goal.object_id])

					try:
						grasp_file = "/tmp/grasp.xml"
						f = open(grasp_file, 'w')
						f.write(resp.msg[0].bs)
						f.close()

						grasps = grasping_functions.getGrasps(grasp_file, all_grasps=True, msg=True)
						GRASPS = grasping_functions.getGraspsByAxis(grasps, server_goal.pose_id)
						os.remove(grasp_file)

						rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.")		

						if len(GRASPS)>0:
							server_result.grasp_configuration = GRASPS
							self.grasp_action_server.set_succeeded(server_result)
						else:
							rospy.logerr("No grasping configurations for this object.")
							server_result.grasp_configuration = []
							self.grasp_action_server.set_aborted(server_result)
		


					except rospy.ServiceException, e:
						rospy.logerr("Service did not process request: %s", str(e))
						rospy.logerr("No grasping configurations for this object.")
						server_result.grasp_configuration = []
						self.grasp_action_server.set_aborted(server_result)

			except rospy.ServiceException, e:
						rospy.logerr("Service did not process request: %s", str(e))
						rospy.logerr("No grasping configurations for this object.")
						server_result.grasp_configuration = []
						self.grasp_action_server.set_aborted(server_result)


		print "Time employed: ",time.time()-x
		print "-------------------------------"


## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('grasp_server')

	SCRIPT = grasp_action_server()
	rospy.loginfo("/grasp_server is running")
	rospy.spin()
