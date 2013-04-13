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
from srs_msgs.msg import GraspingErrorCodes
from cob_kinematics.srv import * #GetPositionIKExtended
from arm_navigation_msgs.srv import * #PlanningScene

class graspingutils():

	def __init__(self, simulation=True):
		self.simulation = simulation
		self.ik_service_name = rospy.get_param("/srs/ik_solver", "/cob_ik_wrapper/arm/get_ik") 
		self.env_service_name = rospy.get_param("/srs/env_planner", "/environment_server/set_planning_scene_diff") 
		self.SetPlanningSceneDiffService = rospy.ServiceProxy(self.env_service_name, SetPlanningSceneDiff)
		self.ik_service = rospy.ServiceProxy(self.ik_service_name, self.get_ik_srv_type()) 
		self.is_grasped_service = rospy.ServiceProxy('/sdh_controller/is_grasped', Trigger)
		self.is_cylindric_grasped_service = rospy.ServiceProxy('/sdh_controller/is_cylindric_grasped', Trigger)
		self.is_grasped_aux = rospy.ServiceProxy('/srs_grasping/is_grasped', Trigger)

	def get_ik_srv_name(self):
		return self.ik_service_name;

	def get_ik_srv_type(self):
		if self.ik_service_name == "/cob_ik_wrapper/arm/get_ik_extended":
			return GetPositionIKExtended()
		elif self.ik_service_name == "/srs_arm_kinematics/get_ik" or self.ik_service_name == "/cob_arm_kinematics/get_ik" or self.ik_service_name == "/cob_ik_wrapper/arm/get_ik":
			return GetPositionIK();
		elif self.ik_service_name == "/srs_arm_kinematics/get_constraint_aware_ik" or self.ik_service_name == "/cob_arm_kinematics/get_constraint_aware_ik" or self.ik_service_name == "/cob_ik_wrapper/arm/get_constraint_aware_ik":
			return GetConstraintAwarePositionIK();

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
			return GraspingErrorCodes.UNKNOWN_CATEGORY;


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
			return GraspingErrorCodes.CORRUPTED_GRASP_FILE;


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

	def parse_cartesian_param(self, param, now = None):
		if now is None:
			now = rospy.Time.now()

		ps = PoseStamped()
		ps.pose.orientation.w = 1.0
		ps.header.stamp = now
		if type(param) is not PoseStamped and param is not None:
			ps.header.frame_id = param[0]
			if len(param) > 1:
				ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
		if len(param) > 2:
			ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
		else:
			ps = param
		return ps

	def parse_cartesian_parameters(self, arm_name, parameters):
		now = rospy.Time.now()

		# parse pose_target
		param = parameters
		second_param = None
		if type(parameters) is list and len(parameters) > 0:
			if type(parameters[0]) is not str:
				param = parameters[0]
				if len(parameters) > 1:
					second_param = parameters[1]

		pose_target = self.parse_cartesian_param(param, now)[0]

		# parse pose_origin
		param = second_param
		ps = PoseStamped()
		ps.pose.orientation.w = 1.0
		ps.header.stamp = pose_target.header.stamp
		ps.header.frame_id = rospy.get_param("/cob_arm_kinematics/"+arm_name+"/tip_name")
		if type(param) is not PoseStamped:
			if param is not None and len(param) >=1:
				ps.header.frame_id = param[0]
				if len(param) > 1:
					ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
				if len(param) > 2:
					ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
		else:
			ps = param
		return pose_target,ps

	def callIKSolver(self,current_pose, goal_pose):

		req = GetPositionIKRequest();
		if self.ik_service_name == "/srs_arm_kinematics/get_ik" or self.ik_service_name == "/cob_arm_kinematics/get_ik" or self.ik_service_name == "/cob_ik_wrapper/arm/get_ik":
			req = GetPositionIKRequest();

		elif self.ik_service_name == "/srs_arm_kinematics/get_constraint_aware_ik" or self.ik_service_name == "/cob_arm_kinematics/get_constraint_aware_ik" or self.ik_service_name == "/cob_ik_wrapper/arm/get_constraint_aware_ik":
			planning_scene_request = SetPlanningSceneDiffRequest()
			planning_scene_response = self.SetPlanningSceneDiffService(planning_scene_request)
			req = GetConstraintAwarePositionIKRequest();

		elif self.ik_service_name == "/cob_ik_wrapper/arm/get_ik_extended":
			planning_scene_request = SetPlanningSceneDiffRequest()
			planning_scene_response = self.SetPlanningSceneDiffService(planning_scene_request)
			ps_target, ps_origin = self.parse_cartesian_parameters('arm', [[goal_pose], ['sdh_palm_link']])
			req = GetPositionIKExtendedRequest()
			req.ik_pose = ps_origin.pose
    			req.constraint_aware = True
    			goal_pose =  ps_target
		else:
			rospy.logerr("Unknown IK service name");
			return ([],-1)

		req.timeout = rospy.Duration(5.0)
		req.ik_request.ik_link_name = "sdh_palm_link"
		req.ik_request.pose_stamped = goal_pose
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.ik_seed_state.joint_state.name = ["arm_%d_joint" % (d+1) for d in range(7)]
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
			rotacion = tf.transformations.euler_matrix(e[0],e[1],-e[2], axes='sxyz');
		else:	
			#real robot
			e = tf.transformations.euler_from_quaternion([obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w],axes='sxyz');
			rotacion = tf.transformations.euler_matrix(e[0],e[1],e[2], axes='sxyz');

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
			resp_aux = self.is_grasped_aux()
			response = resp1.success.data or resp2.success.data or resp_aux.success.data
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return GraspingErrorCodes.SERVICE_DID_NOT_PROCESS_REQUEST

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
			return GraspingErrorCodes.UNKNOWN_CATEGORY
		
		return t2;


	def set_pregrasp_offsets(self, category, pre, pregrasps_offsets):
		if len(pregrasps_offsets) != 2:
			return pre;

		os = 0.6;
		if category=="FRONT":
			o = pregrasps_offsets[0] - os;
			offset = (o, 0)[o<=0];
			pre.position.x += offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		elif category=="SIDE":
			o = pregrasps_offsets[0] - os;
			offset = (o, 0)[o<=0];
			pre.position.y += offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		elif category=="-SIDE":
			o = pregrasps_offsets[0] - os;
			offset = (o, 0)[o<=0];
			pre.position.y -= offset;
			if pregrasps_offsets[1] > 0.0:
				pre.position.z += pregrasps_offsets[1];
		else: #category=="TOP":
			o = pregrasps_offsets[1] - os;
			offset = (o, 0)[o<=0];
			pre.position.z += offset;

		return pre;
