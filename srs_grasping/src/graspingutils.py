#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import tf
import numpy

from xml.dom import minidom
from srs_msgs.msg import *
from kinematics_msgs.srv import *
from cob_srvs.srv import *
from geometry_msgs.msg import *
from numpy import *

class graspingutils():

	def __init__(self, simulation=True):

		self.simulation = simulation;
		self.ik_service = rospy.ServiceProxy('/arm_kinematics/get_constraint_aware_ik', GetConstraintAwarePositionIK)
		self.is_grasped_service = rospy.ServiceProxy('/sdh_controller/is_grasped', Trigger)
		self.is_cylindric_grasped_service = rospy.ServiceProxy('/sdh_controller/is_cylindric_grasped', Trigger)



	def joint_filter(self, finalconfig):
		OR = finalconfig[7:14]
		fc = [OR[2], OR[3], OR[4], OR[0], OR[1], OR[5], OR[6]]
		if (fc[1]>=-1.15 and fc[3]>=-1.15 and fc[5]>=-1.25 and fc[1]<=1 and fc[3]<=1 and fc[5]<=1 and fc[2]>-0.5 and fc[4]>-0.5 and fc[6]>-0.5):
			return (True, [fc[0], fc[5], fc[6], fc[1], fc[2], fc[3], fc[4]]);
		else:
			return (False, []);
		
	def get_grasping_direction(self,matrix):

		x = (matrix)[0][2]
		y = (matrix)[1][2]
		z = (matrix)[2][2]

		if x > 0.85:
			return "X"

		elif x < -0.85:
			return "-X"

		elif y > 0.85:
			return "Y"

		elif y < -0.85:
			return "-Y"

		elif z > 0.85:
			return "Z"

		elif z < -0.85:
			return "-Z"

		else:
			return -1


	def valid_grasp(self, category):
		return ((category == "TOP") or (category == "SIDE") or (category == "-SIDE") or (category == "FRONT"));


	def get_grasps(self, file_name):

		try:
			xmldoc = minidom.parse(file_name)  
			object_id = int(((xmldoc.firstChild)).getElementsByTagName('object_id')[0].firstChild.nodeValue)
			res = []
			hijos = (xmldoc.firstChild).getElementsByTagName('Grasp')
			grasps = []
			for hijo in hijos:
				sdh_joint_values = ((hijo.getElementsByTagName('joint_values'))[0]).firstChild.nodeValue
				aux = (hijo.getElementsByTagName('GraspPose'))[0]
				Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
				Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)
				grasp = PoseStamped()
				grasp.header.frame_id = "/base_link";
				grasp.pose.position.x = float(Translation[0])
				grasp.pose.position.y = float(Translation[1])
				grasp.pose.position.z = float(Translation[2])
				Rotation =  tf.transformations.quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
				grasp.pose.orientation.x = float(Rotation[0])
				grasp.pose.orientation.y = float(Rotation[1])
				grasp.pose.orientation.z = float(Rotation[2])
				grasp.pose.orientation.w = float(Rotation[3])
				aux = (hijo.getElementsByTagName('PreGraspPose'))[0]

				Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
				Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)

				pre_grasp = PoseStamped()
				pre_grasp.header.frame_id = "/base_link"; 
				pre_grasp.pose.position.x = float(Translation[0])
				pre_grasp.pose.position.y = float(Translation[1])
				pre_grasp.pose.position.z = float(Translation[2])
				Rotation = tf.transformations.quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
				pre_grasp.pose.orientation.x = float(Rotation[0])
				pre_grasp.pose.orientation.y = float(Rotation[1])
				pre_grasp.pose.orientation.z = float(Rotation[2])
				pre_grasp.pose.orientation.w = float(Rotation[3])
				category = ((hijo.getElementsByTagName('category'))[0]).firstChild.nodeValue

				GC = DBGrasp()
				GC.object_id = object_id
				GC.hand_type = "SDH"
				GC.sdh_joint_values = eval(sdh_joint_values)
				GC.pre_grasp = pre_grasp
				GC.grasp = grasp
				GC.category= str(category)
				grasps.append(GC)

			return grasps
		except:
			print "There are not generated files for this object."
			return -1


	def get_grasp_category(self, pre, g):

		x = pre.x - g.x;
		y = pre.y - g.y;
		z = pre.z - g.z;

		if (x > 0.08):
			category = "FRONT";
		elif (x < -0.08):
			category = "BACK";
		elif (y > 0.08):
			category = "SIDE";
		elif (y < -0.08):
			category = "-SIDE";
		elif (z > 0.08):
			category = "TOP";
		elif (z < -0.08):
			category = "DOWN";
		else:
			category = "UNKNOWN";

		return category;


	def callIKSolver(self,current_pose, goal_pose):
		req = GetConstraintAwarePositionIKRequest();
		req.ik_request.ik_link_name = "sdh_palm_link";
		req.ik_request.ik_seed_state.joint_state.position = current_pose;
		req.ik_request.pose_stamped = goal_pose;
		resp = self.ik_service(req);
		return (list(resp.solution.joint_state.position), resp.error_code);


	def matrix_from_pose(self, gp): 
		return numpy.matrix(self.array_from_pose(gp))


	def pose_from_matrix(self, matrix):
		t = tf.transformations.translation_from_matrix(matrix);
		q = tf.transformations.quaternion_from_matrix(matrix);

		ps = PoseStamped();
		ps.header.stamp = rospy.Time.now();
		ps.header.frame_id = "/base_link";
		ps.pose.position.x = t[0];
		ps.pose.position.y = t[1];
		ps.pose.position.z = t[2];
		ps.pose.orientation.x = q[0];
		ps.pose.orientation.y = q[1];
		ps.pose.orientation.z = q[2];
		ps.pose.orientation.w = q[3];
		return ps;

		
	def array_from_pose(self, gp): 

		q = []
		q.append(gp.orientation.x)
		q.append(gp.orientation.y)
		q.append(gp.orientation.z)
		q.append(gp.orientation.w)
		e = tf.transformations.euler_from_quaternion(q, axes='sxyz')

		matrix = tf.transformations.euler_matrix(e[0],e[1],e[2] ,axes='sxyz')
		matrix[0][3] = gp.position.x
		matrix[1][3] = gp.position.y
		matrix[2][3] = gp.position.z
		return matrix


	def rotation_matrix(self, obj):

		if self.simulation:
			#hack for gazebo simulation
			e = tf.transformations.euler_from_quaternion([obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w],axes='sxzy');
			rotacion =  tf.transformations.euler_matrix(e[0],e[1],-e[2], axes='sxyz');
		else:	
			#real robot
			e = tf.transformations.euler_from_quaternion([obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w],axes='sxyz');
			rotacion =  tf.transformations.euler_matrix(e[0],e[1],e[2], axes='sxyz');

		rotacion[0,3] = obj.position.x;
		rotacion[1,3] = obj.position.y;
		rotacion[2,3] = obj.position.z;

		return rotacion;

	def OR_to_COB(self, values):
		OR = values[7:14]
		values = [OR[2], OR[3], OR[4], OR[0], OR[1], OR[5], OR[6]]
		return values

	def COB_to_OR(self, values):

		res = [0 for i in range(28)]	
		values = [values[5], values[6], values[0], values[3], values[4], values[1], values[2]]

		for i in range (0,len(values)):
			res[i+7] = float(values[i])
		return eval(str(res))

	def sdh_tactil_sensor_result(self):
		rospy.loginfo("Reading SDH tactil sensors...");

		try:
			resp1 = self.is_grasped_service()
			resp2 = self.is_cylindric_grasped_service()
			response = resp1.success.data or resp2.success.data
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		return response

	def set_pregrasp(self, t, category, pregrasp_offset):
		t2 = [t[0], t[1], t[2]]

		if category == "X":
			t2[0] -= pregrasp_offset
		elif category == "-X": 
			t2[0] += pregrasp_offset
		elif category == "Y": 
			t2[1] -= pregrasp_offset
		elif category == "-Y": 
			t2[1] += pregrasp_offset
		elif category == "Z":
			t2[2] -= pregrasp_offset 
		elif category == "-Z":
			t2[2] += pregrasp_offset
		else: 
			return -1
		
		return t2;


	def set_pregrasp_offsets(self, category, pre, pregrasps_offsets):
		if len(pregrasps_offsets) != 2:
			return pre;

		if category=="FRONT":
			o = pregrasps_offsets[0] - 0.2;
			offset = (o, 0)[o<=0];
			pre.position.x += offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		elif category=="SIDE":
			o = pregrasps_offsets[0] - 0.2;
			offset = (o, 0)[o<=0];
			pre.position.y += offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		elif category=="-SIDE":
			o = pregrasps_offsets[0] - 0.2;
			offset = (o, 0)[o<=0];
			pre.position.y -= offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		else: #category=="TOP":
			o = pregrasps_offsets[1] - 0.2;
			offset = (o, 0)[o<=0];
			pre.position.z += offset;


		return pre;

