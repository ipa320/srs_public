#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import openravepy
import time
import sys
import tf
import os
#import multiprocessing

from numpy import *

class openraveutils():

	def __init__(self, databaseutils=None, 
			   robotName="care-o-bot3.zae", 
			   robotManipulator="arm",
			   env=None,
			   robot=None,
			   pregrasp_offset=0.2):

		self.robotName = roslib.packages.get_pkg_dir("srs_grasping")+"/robots/"+robotName;
		self.manipulator = robotManipulator;
		self.env = env;
		self.robot = robot;
		self.pregrasp_offset = pregrasp_offset;
		self.databaseutils = databaseutils


		if self.databaseutils is None:
			self.databaseutils = grasping_functions.databaseutils();

		self.graspingutils = self.databaseutils.get_graspingutils();

		try:
			if self.env is None:
				self.env = openravepy.Environment();
			if self.robot is None:
				self.robot = self.env.ReadRobotXMLFile(self.robotName);
			
			self.robot.SetActiveManipulator(self.manipulator);
			self.env.AddRobot(self.robot);
		except:
			rospy.logerr("The robot file %s does not exists.", self.robotName);
			return -1;
			

	def init_env(self,object_id):
		mesh_file = self.databaseutils.get_mesh(object_id);
		if mesh_file is -1:
			return -1;

		try:
			target = self.env.ReadKinBodyXMLFile(mesh_file)
			self.env.AddKinBody(target);
			os.remove(mesh_file);
		except:
			rospy.logerr("The mesh data does not exist or does not work correctly.");
			os.remove(mesh_file);
			return -1;

		return openravepy.databases.grasping.GraspingModel(robot=self.robot,target=target);


	def generate_grasp_file(self, object_id):

		gmodel = self.init_env(object_id);
		if gmodel is -1:
			return -1

		self.generate_grasps(gmodel);

		f_name = "/tmp/"+str(object_id)+".xml"

		f = open(f_name,'w')
		f.write("<?xml version=\"1.0\" ?>\n")
		f.write("<GraspList>\n")
		f.write("<object_id>"+str(object_id)+"</object_id>\n")
		f.write("<hand_type>SDH</hand_type>\n")
		f.write("<joint_names>[sdh_knuckle_joint, sdh_thumb_2_joint, sdh_thumb_3_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint]</joint_names>\n")
		f.write("<tip_link>sdh_palm_link</tip_link>\n")

		cont = 0
		for i in range(0,len(gmodel.grasps)):
			grasp = gmodel.grasps[i]
			print '\r[%s%%][%s/%s] Adding grasps to the new XML file...' % (int((float(float((i+1))/float(len(gmodel.grasps))))*100),(i+1),len(gmodel.grasps)),
			sys.stdout.flush()	
			try:
	 			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
				(err, values) = self.graspingutils.joint_filter(finalconfig[0]);
				if not err:
					continue

				self.robot.GetController().Reset(0)
				self.robot.SetDOFValues(finalconfig[0])
				self.robot.SetTransform(finalconfig[1])
				self.env.UpdatePublishedBodies()
				index = (self.robot.GetLink("sdh_palm_link")).GetIndex()
				matrix = (self.robot.GetLinkTransformations())[index]
				t = tf.transformations.translation_from_matrix(matrix)
				e = tf.transformations.euler_from_matrix(matrix, axes='sxyz')

				category = self.graspingutils.get_grasping_direction(matrix)
				if category is -1:
					continue

				tp = []
				tp = self.graspingutils.set_pregrasp(t, category, self.pregrasp_offset);
				if tp is -1:
					continue;

			   	f.write("<Grasp Index=\""+str(cont)+"\">\n")
				f.write("<joint_values>"+str(values)+"</joint_values>\n")
			   	f.write("<GraspPose>\n")
				f.write("<Translation>["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]</Translation>\n")
				f.write("<Rotation>["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]</Rotation>\n")
			   	f.write("</GraspPose>\n")
		   		f.write("<PreGraspPose>\n")
				f.write("<Translation>["+str(tp[0])+", "+str(tp[1])+", "+str(tp[2])+"]</Translation>\n")
				f.write("<Rotation>["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]</Rotation>\n")
		   		f.write("</PreGraspPose>\n")
				f.write("<category>"+category+"</category>\n")
			   	f.write("</Grasp>\n")
				
				cont += 1
			except:
				continue


		f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n")
		f.write("</GraspList>")
		f.close()
	
		f = open(f_name, 'r')
		res = f.read()
		f.close()
		os.remove(f_name)

		print "%d grasps have been added to the XML file." %cont
		return res;


	def generate_grasps(self, gmodel):
		x = time.time();
		if not gmodel.load():
			rospy.loginfo("GENERATING GRASPS...")
			gmodel.numthreads = 1 #multiprocessing.cpu_count()
			gmodel.autogenerate()
			rospy.loginfo("GRASPS GENERATION HAS FINISHED. Time employed: %s", str(time.time() - x))


	def generator(self, object_id):
		grasps = self.generate_grasp_file(object_id)
		if grasps is -1:
			return -1;

		if self.databaseutils.insert_grasps(object_id, grasps) is -1:
			return -1;

		return 0;

	def show_all_grasps(self, object_id, grasps):		#Group of grasps (grasp of GraspConfiguration)

		gmodel = self.init_env(object_id);
		if gmodel is -1:
			return -1

		self.init_simulator();
		manip = self.robot.GetManipulator(self.manipulator)
		with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):
			for i in range(0,len(grasps)):
				grasp = grasps[i]
				print 'grasp %d/%d'%(i,len(grasps))

				dof_values = self.graspingutils.COB_to_OR(grasp.sdh_joint_values);
				self.robot.SetDOFValues(dof_values);
				Tgrasp = self.graspingutils.array_from_pose(grasp.grasp.pose)
				index = (self.robot.GetLink("sdh_palm_link")).GetIndex()
				matrix = (self.robot.GetLinkTransformations())[index]
				Tdelta = dot(Tgrasp,linalg.inv(matrix))
				for link in manip.GetChildLinks():
					link.SetTransform(dot(Tdelta,link.GetTransform()))
				self.env.UpdatePublishedBodies()

				raw_input('Next config.')
		return 0;

	def grasp_view(self, object_id, grasp, object_pose):	#Individual grasp (grasp of GraspSubConfiguration)

		gmodel = self.init_env(object_id);
		if gmodel is -1:
			return -1

		self.init_simulator();


		m = self.graspingutils.matrix_from_pose(grasp.grasp);
		rot = self.graspingutils.matrix_from_pose(self.graspingutils.rotation_matrix(object_pose));
		sol = rot.I * m;
		t = tf.transformations.translation_from_matrix(sol)
		q = tf.transformations.quaternion_from_matrix(sol)
		grasp.grasp.position.x = t[0]
		grasp.grasp.position.y = t[1]
		grasp.grasp.position.z = t[2] 
		grasp.grasp.orientation.x = q[0]
		grasp.grasp.orientation.y = q[1]
		grasp.grasp.orientation.z = q[2]
		grasp.grasp.orientation.w = q[3]

		manip = self.robot.GetManipulator(self.manipulator)

		with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):
			dof_values = self.graspingutils.COB_to_OR(grasp.sdh_joint_values)
			self.robot.SetDOFValues(dof_values)
			Tgrasp = array_from_pose(grasp.grasp)
			index = (self.robot.GetLink("sdh_palm_link")).GetIndex()
			matrix = (self.robot.GetLinkTransformations())[index]
			Tdelta = dot(Tgrasp,linalg.inv(matrix))
			for link in manip.GetChildLinks():
				link.SetTransform(dot(Tdelta,link.GetTransform()))
			self.env.UpdatePublishedBodies()
			raw_input("Continue...");
		return 0;


	def init_simulator(self):
		self.env.SetViewer('qtcoin');
		time.sleep(1.0)

