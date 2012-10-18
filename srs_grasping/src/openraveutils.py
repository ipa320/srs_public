#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import openravepy
import time
import sys
import tf
import os
import copy
#import multiprocessing

from numpy import *
from traceback import print_exc
from xml.dom.minidom import Document
import re
from srs_msgs.msg import GraspingErrorCodes
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
			return GraspingErrorCodes.CORRUPTED_ROBOT_MESH_FILE;
			

	def init_env(self,object_id):
		mesh_file = self.databaseutils.get_mesh(object_id);
		if mesh_file < 0:
			return mesh_file

		try:
			target = self.env.ReadKinBodyXMLFile(mesh_file)
			self.env.AddKinBody(target);
			os.remove(mesh_file);
		except:
			rospy.logerr("The mesh data does not work correctly.");
			os.remove(mesh_file);
			return GraspingErrorCodes.CORRUPTED_MESH_FILE;

		return openravepy.databases.grasping.GraspingModel(robot=self.robot,target=target);


	def generate_grasp_file_OLD(self, object_id):

		gmodel = self.init_env(object_id);
		if gmodel < 0:
			return gmodel

		self.generate_grasps_OLD(gmodel);

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
				if category is GraspingErrorCodes.UNKNOWN_CATEGORY:
					continue

				tp = []
				tp = self.graspingutils.set_pregrasp(t, category, self.pregrasp_offset);
				if tp is GraspingErrorCodes.UNKNOWN_CATEGORY:
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


	def generate_grasps_OLD(self, gmodel):
		x = time.time();
		if not gmodel.load():
			rospy.loginfo("GENERATING GRASPS...")
			gmodel.numthreads = 1 #multiprocessing.cpu_count()
			gmodel.autogenerate()
			rospy.loginfo("GRASPS GENERATION HAS FINISHED. Time employed: %s", str(time.time() - x))

	def generator_OLD(self, object_id):
		grasps = self.generate_grasp_file_OLD(object_id)
		if grasps < 0:
			return grasps;

		return self.databaseutils.insert_grasps(object_id, grasps);


	def show_all_grasps(self, object_id, grasps):		#Group of grasps (grasp of GraspConfiguration)

		gmodel = self.init_env(object_id);
		if gmodel < 0:
			return gmodel

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
		return GraspingErrorCodes.SUCCESS;


	def grasp_view(self, object_id, grasp, object_pose):	#Individual grasp (grasp of GraspSubConfiguration)

		gmodel = self.init_env(object_id);
		if gmodel < 0:
			return gmodel

		self.init_simulator();

		m = self.graspingutils.matrix_from_pose(grasp.grasp.pose);
		rot = self.graspingutils.matrix_from_pose(object_pose);
		sol = rot.I * m;
		t = tf.transformations.translation_from_matrix(sol)
		q = tf.transformations.quaternion_from_matrix(sol)

		grasp.grasp.pose.position.x = t[0]
		grasp.grasp.pose.position.y = t[1]
		grasp.grasp.pose.position.z = t[2] 
		grasp.grasp.pose.orientation.x = q[0]
		grasp.grasp.pose.orientation.y = q[1]
		grasp.grasp.pose.orientation.z = q[2]
		grasp.grasp.pose.orientation.w = q[3]

		manip = self.robot.GetManipulator(self.manipulator)

		with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):
			dof_values = self.graspingutils.COB_to_OR(grasp.sdh_joint_values)
			self.robot.SetDOFValues(dof_values)
			Tgrasp = self.graspingutils.array_from_pose(grasp.grasp.pose)
			index = (self.robot.GetLink("sdh_palm_link")).GetIndex()
			matrix = (self.robot.GetLinkTransformations())[index]
			Tdelta = dot(Tgrasp,linalg.inv(matrix))
			for link in manip.GetChildLinks():
				link.SetTransform(dot(Tdelta,link.GetTransform()))
			self.env.UpdatePublishedBodies()
			time.sleep(10.0)
		return GraspingErrorCodes.SUCCESS;


	def init_simulator(self):
		self.env.SetViewer('qtcoin');
		time.sleep(1.0)


########################## NEW FUNCTIONS ########################################################

	def generator(self, object_id):
		grasps = self.generate_grasp_file(object_id)
		if grasps < 0:
			return grasps;
		elif self.databaseutils.insert_grasps(object_id, grasps) < 0:
			return GraspingErrorCodes.SERVICE_DID_NOT_PROCESS_REQUEST;
		else:
			return GraspingErrorCodes.SUCCESS;


	def generate_grasps(self, gmodel, doc):
		starttime = time.time();
		rospy.loginfo("GENERATING GRASPS...")
		counter = self.autogenerate(gmodel, doc);
		rospy.loginfo("GRASPS GENERATION HAS FINISHED. Time employed: %s", str(time.time() - starttime))
		return counter;
		

	def autogenerate(self, gmodel, doc, options=None):
		counter = self.generate(gmodel, doc, *gmodel.autogenerateparams(options))
		starttime = time.time();
		gmodel.save()
		return counter


	def generate(self, gmodel, doc, *args, **kwargs):
		graspingnoise = args[4]
		forceclosure = args[5]
		forceclosurethreshold = args[6]
		checkgraspfn = args[7]

		starttime = time.time()
		statesaver = self.robot.CreateRobotStateSaver()
		bodies = [(b,b.IsEnabled()) for b in self.env.GetBodies() if b != self.robot and b != gmodel.target]
		if gmodel.disableallbodies:
		    for b in bodies:
			b[0].Enable(False)
		try:
		    if gmodel.numthreads is not None and gmodel.numthreads > 1:
			gmodel._generateThreaded(*args,**kwargs)
		    else:
			with gmodel.GripperVisibility(gmodel.manip):
			    if self.env.GetViewer() is not None:
				self.env.UpdatePublishedBodies()

			    aux = [item for item in args]
			    aux[1] = array([0.03, 0.05])	#standoffs
				
			    producer,consumer,gatherer,numjobs = gmodel.generatepcg(*aux,**kwargs)
			    counter = 0
			    counter_isValid = 0
			    for work in producer():
				print '\r[grasp %d/%d][Good grasps: %d]'%(counter,numjobs,counter_isValid),
				sys.stdout.flush()
				counter += 1
				results = self.consumer(gmodel, doc, counter_isValid, graspingnoise, forceclosure, forceclosurethreshold, checkgraspfn, *work)
				if results is not None:
				    gatherer(results)
				    counter_isValid+=1;

			    gatherer() # gather results
		finally:
		    for b,enable in bodies:
			b.Enable(enable)
		    statesaver = None

		print 'grasping finished in %fs'%(time.time()-starttime)
		return counter_isValid


	def consumer(self, gmodel, doc, counter, graspingnoise, forceclosure, forceclosurethreshold, checkgraspfn, approachray, roll, preshape, standoff, manipulatordirection):

		grasp = zeros(gmodel.totaldof)
		grasp[gmodel.graspindices.get('igrasppos')] = approachray[0:3]
		grasp[gmodel.graspindices.get('igraspdir')] = -approachray[3:6]
		grasp[gmodel.graspindices.get('igrasproll')] = roll
		grasp[gmodel.graspindices.get('igraspstandoff')] = standoff
		grasp[gmodel.graspindices.get('igrasppreshape')] = preshape
		grasp[gmodel.graspindices.get('imanipulatordirection')] = manipulatordirection

		try:
			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=grasp,graspingnoise=graspingnoise,translate=True,forceclosure=forceclosure,forceclosurethreshold=forceclosurethreshold)

			(err, values) = self.graspingutils.joint_filter(finalconfig[0]);
			if not err:
				return None
		except:
			print 'Grasp Failed: '
                	print_exc(e)
			return None

		Tlocalgrasp = eye(4)
		with self.robot:
			self.robot.SetTransform(finalconfig[1])
			Tgrasp = gmodel.manip.GetEndEffectorTransform()
			Tlocalgrasp = dot(linalg.inv(gmodel.target.GetTransform()),Tgrasp)
			# find a non-colliding transform
			gmodel.setPreshape(grasp)
			direction = gmodel.getGlobalApproachDir(grasp)
			Tgrasp_nocol = array(Tgrasp)
			while gmodel.manip.CheckEndEffectorCollision(Tgrasp_nocol):
				Tgrasp_nocol[0:3,3] -= direction*gmodel.collision_escape_offset
			Tlocalgrasp_nocol = dot(linalg.inv(gmodel.target.GetTransform()),Tgrasp_nocol)
			self.robot.SetDOFValues(finalconfig[0])
			if self.env.GetViewer() is not None:
				gmodel.contactgraph = gmodel.drawContacts(contacts) if len(contacts) > 0 else None
				self.env.UpdatePublishedBodies()
			grasp[gmodel.graspindices.get('igrasptrans')] = reshape(transpose(Tlocalgrasp[0:3,0:4]),12)
			grasp[gmodel.graspindices.get('grasptrans_nocol')] = reshape(transpose(Tlocalgrasp_nocol[0:3,0:4]),12)
			grasp[gmodel.graspindices.get('forceclosure')] = mindist if mindist is not None else 0
			self.robot.SetTransform(self.robot.GetTransform()) # transform back to original position for checkgraspfn
			if not forceclosure or mindist >= forceclosurethreshold:
				grasp[gmodel.graspindices.get('performance')] = gmodel._ComputeGraspPerformance(grasp, graspingnoise=graspingnoise,translate=True,forceclosure=False)

				if checkgraspfn is None or gmodel.checkgraspfn(contacts,finalconfig,grasp,{'mindist':mindist,'volume':volume}):

					if self.add_valid_grasp_to_file(doc, counter, values):
						return grasp
					else:
						return None

			return None


	def generate_grasp_file(self, object_id):
	
		gmodel = self.init_env(object_id);
		if gmodel < 0:
			return gmodel

		doc = Document()
		GraspList = doc.createElement("GraspList")
		doc.appendChild(GraspList)

		obj_id = doc.createElement("object_id")
		GraspList.appendChild(obj_id)
		obj_id_text = doc.createTextNode(str(object_id))
		obj_id.appendChild(obj_id_text)

		hand_type = doc.createElement("hand_type")
		GraspList.appendChild(hand_type)
		hand_type_text = doc.createTextNode("SDH")
		hand_type.appendChild(hand_type_text)

		joint_names = doc.createElement("joint_names")
		GraspList.appendChild(joint_names)
		joint_names_text = doc.createTextNode("[sdh_knuckle_joint, sdh_thumb_2_joint, sdh_thumb_3_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint]")
		joint_names.appendChild(joint_names_text)

		tip_link = doc.createElement("tip_link")
		GraspList.appendChild(tip_link)
		tip_link_text = doc.createTextNode("sdh_palm_link")
		tip_link.appendChild(tip_link_text)

		cont = self.generate_grasps(gmodel, doc);

		NumberOfGrasps = doc.createElement("NumberOfGrasps")
		GraspList.appendChild(NumberOfGrasps)
		NumberOfGrasps_text = doc.createTextNode(str(cont))
		NumberOfGrasps.appendChild(NumberOfGrasps_text)

		uglyXml = doc.toprettyxml(indent="  ")
		text_re = re.compile('>\n\s+([^<>\s].*?)\n\s+</', re.DOTALL)    
		prettyXml = text_re.sub('>\g<1></', uglyXml)

		print str(cont)+" grasps have been added to the XML file."
		return prettyXml;


	def add_valid_grasp_to_file(self, doc, counter, values):

		index = (self.robot.GetLink("sdh_palm_link")).GetIndex()
		matrix = (self.robot.GetLinkTransformations())[index]
		t = tf.transformations.translation_from_matrix(matrix)
		e = tf.transformations.euler_from_matrix(matrix, axes='sxyz')

		category = self.graspingutils.get_grasping_direction(matrix)
		if category is GraspingErrorCodes.UNKNOWN_CATEGORY:
			return False;

		tp = []
		tp = self.graspingutils.set_pregrasp(t, category, self.pregrasp_offset);
		if tp is GraspingErrorCodes.UNKNOWN_CATEGORY:
			return False;

		GraspList = (doc.getElementsByTagName("GraspList"))[0]
		Grasp = doc.createElement("Grasp")
		Grasp.setAttribute("Index", str(counter))
		GraspList.appendChild(Grasp)
		
		joint_values = doc.createElement("joint_values")
		Grasp.appendChild(joint_values)
		joint_values_text = doc.createTextNode(str(values))
		joint_values.appendChild(joint_values_text)

		GraspPose = doc.createElement("GraspPose")
		Grasp.appendChild(GraspPose)
		Translation = doc.createElement("Translation")
		GraspPose.appendChild(Translation)
		Translation_text = doc.createTextNode("["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]")
		Translation.appendChild(Translation_text)
		Rotation = doc.createElement("Rotation")
		GraspPose.appendChild(Rotation)
		Rotation_text = doc.createTextNode("["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]")
		Rotation.appendChild(Rotation_text)

		PreGraspPose = doc.createElement("PreGraspPose")
		Grasp.appendChild(PreGraspPose)
		Translation2 = doc.createElement("Translation")
		PreGraspPose.appendChild(Translation2)
		Translation2_text = doc.createTextNode("["+str(tp[0])+", "+str(tp[1])+", "+str(tp[2])+"]")
		Translation2.appendChild(Translation2_text)
		Rotation2 = doc.createElement("Rotation")
		PreGraspPose.appendChild(Rotation2)
		Rotation2_text = doc.createTextNode("["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]")
		Rotation2.appendChild(Rotation2_text)

		cat = doc.createElement("category")
		Grasp.appendChild(cat)
		category_text = doc.createTextNode(category)
		cat.appendChild(category_text)

		return True;
