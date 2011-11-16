#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import openravepy

import time
import sys
import os

from srs_object_database.msg import *
from srs_object_database.srv import *

import grasping_functions
##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):

		self.robotName = 'robots/care-o-bot3.zae'


	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self, object_id):

		env = openravepy.Environment()

		try:
	    		robot = env.ReadRobotXMLFile(self.robotName)
			env.AddRobot(robot)
		except:
			rospy.logerr("The robot file %s does not exists.", self.robotName)
			return -1


		rospy.loginfo("Waiting for /get_model_mesh service...")
		rospy.wait_for_service('/get_model_mesh')
		rospy.loginfo("/get_model_mesh service found!")

		get_mesh = rospy.ServiceProxy('/get_model_mesh', GetMesh)

		try:
			resp = get_mesh(model_ids=[object_id])
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		try:

			mesh_file = "/tmp/mesh.iv"
			f = open(mesh_file, 'w')
			res = f.write(resp.msg[0].data)
			f.close()

			target = env.ReadKinBodyXMLFile(mesh_file)
			env.AddKinBody(target)
			os.remove(mesh_file)
		except:
			rospy.logerr("The mesh data does not exist or does not work correctly.")
			os.remove(mesh_file)
			return -1


		robot.SetActiveManipulator("arm")		#care-o-bot3.zae
		gmodel = openravepy.databases.grasping.GraspingModel(robot=robot,target=target)
		if not gmodel.load():
			rospy.loginfo("GENERATING GRASPS...")
			gmodel.autogenerate()
			rospy.loginfo("GENERATING GRASPS HAS FINISHED.")


		grasping_functions.generateFile(targetName=str(object_id), gmodel=gmodel, env=env)
		grasp_file = "/tmp/"+str(object_id)+".xml"
		f = open(grasp_file, 'r')
		res = f.read()
		f.close()
		os.remove(grasp_file)


		#Insertar res en db
		rospy.loginfo("Waiting for /insert_object_service...")
		rospy.wait_for_service('/insert_object_service')
		rospy.loginfo("/insert_object_service found!")

		insert_obj = rospy.ServiceProxy('/insert_object_service', InsertObject)
		try:
			resp = insert_obj(model_id=object_id, data_grasp=res)
		except rospy.ServiceException, e:
		  	rospy.logerr("Service did not process request: %s", str(e))
			return -1
		
		



##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	#rospy.init_node('grasp_generator')
	s = SCRIPT()
    	s.run(1)
