#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import os
import grasping_functions

from srs_object_database_msgs.srv import *
from srs_grasping.srv import *

class databaseutils():

	def __init__(self, graspingutils=None):

		self.graspingutils = graspingutils;
		self.get_mesh_service = rospy.ServiceProxy('/get_model_mesh', GetMesh);
		self.get_grasps_service = rospy.ServiceProxy('/get_model_grasp', GetGrasp);
		self.get_object_info_service = rospy.ServiceProxy('/get_models', GetObjectId);
		self.insert_obj_service = rospy.ServiceProxy('/insert_object_service', InsertObject)

		if self.graspingutils is None:
			self.graspingutils = grasping_functions.graspingutils();

	def get_mesh(self, object_id):
		try:
			resp = self.get_mesh_service(model_ids=[object_id])
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		if len(resp.msg) == 0:
			rospy.logerr("The object with ID=%d has not mesh file in the DB", object_id);
			return -1

		mesh_file = "/tmp/mesh.iv"
		f = open(mesh_file, 'w')
		res = f.write(resp.msg[0].data)
		f.close()

		return mesh_file

	def insert_grasps(self, object_id, grasp_file):
		try:
			resp = self.insert_obj_service(model_id=object_id, data_grasp=grasp_file)
		except rospy.ServiceException, e:
		  	rospy.logerr("Service did not process request: %s", str(e))
			return -1

		return 0;

	def get_grasps(self, object_id):

		server_result = GetDB_GraspsResponse();
		
		try:
			resp = self.get_grasps_service(model_ids=[object_id]);
		except rospy.ServiceException, e:
		  	rospy.logerr("Service did not process request: %s", str(e))
			return -1
		

		if len(resp.msg) == 0:
			rospy.loginfo("No grasping data for this object.");	
			return -1;

		try:
			grasp_file = "/tmp/grasp.xml";
			f = open(grasp_file, 'w');
			f.write(resp.msg[0].bs);
			f.close();

			GRASPS = self.graspingutils.get_grasps(grasp_file);
			if GRASPS == -1:
				rospy.logerr("ERROR reading the grasp file");
				return -1;

			os.remove(grasp_file);

			rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.");	

			server_result.grasp_configuration = GRASPS;
			return server_result;

		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e));
			rospy.logerr("No grasping data for this object.");
			return -1;

	def get_object_id(self, object_name):
		try:
			resp = self.get_object_info_service("name", object_name)
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		if len(resp.model_ids) == 0:
			rospy.logerr("No info for this object.");
			return -1;

		return resp.model_ids[0];

	def get_object_name(self, object_id):
		try:
			resp = self.get_object_info_service("id", str(object_id))
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		if len(resp.model_desc) == 0:
			rospy.logerr("No info for this object.");
			return -1;

		return resp.model_desc[0];
	
	def get_graspingutils(self):
		return self.graspingutils;
