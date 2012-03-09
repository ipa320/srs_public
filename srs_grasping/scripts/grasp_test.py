#!/usr/bin/python
import roslib
roslib.load_manifest('srs_grasping')

import time
import rospy
import tf
import math
import random
import scipy

from geometry_msgs.msg import *
from srs_grasping.msg import *
from srs_grasping.srv import *
from kinematics_msgs.srv import *

from simple_script_server import *

from numpy import matrix

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from srs_object_database.msg import *
from srs_object_database.srv import *

import grasping_functions
from srs_grasping.srv import *

class GraspScript(script):
		
	def __init__(self):
		rospy.loginfo("Waiting /arm_kinematics/get_ik service...")
		rospy.wait_for_service('/arm_kinematics/get_ik')
		rospy.loginfo("/arm_kinematics/get_ik has been found!")

		rospy.loginfo("Waiting /get_grasp_configurations service...")
		rospy.wait_for_service('/get_grasp_configurations')
		rospy.loginfo("/get_grasp_configurations has been found!")

		rospy.loginfo("Waiting /get_grasps_from_position service...")
		rospy.wait_for_service('/get_grasps_from_position')
		rospy.loginfo("/get_grasps_from_position has been found!")


		self.sss = simple_script_server()
		self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)

		"""
		# initialize components (not needed for simulation)
		self.sss.init("tray")
		self.sss.init("torso")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("base")
		
		# move to initial positions
		handle_arm = self.sss.move("arm","folded",False)
		handle_torso = self.sss.move("torso","home",False)
		handle_sdh = self.sss.move("sdh","home",False)
		self.sss.move("tray","down")
		handle_arm.wait()
		handle_torso.wait()
		handle_sdh.wait()

		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()
		"""

	def execute(self):
		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2)

		# prepare for grasping
		#self.sss.move("base","kitchen")
		#self.sss.move("arm","look_at_table")
		self.sss.move("sdh","cylopen")

		#current_joint_configuration
		sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
		while sub.get_num_connections() == 0:
			time.sleep(0.3)
			continue
		print "--"
		#Detect
		"""
		self.srv_name_object_detection = '/object_detection/detect_object'
		detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
		req = DetectObjectsRequest()
		req.object_name.data = "milk"
		res = detector_service(req)

		for i in range(0,len(res.object_list.detections)):
			print str(i)+": "+res.object_list.detections[i].label
		
		index = 0;#-1
		while (index < 0):
			index = int(raw_input("Select object to grasp: "))
		
		obj = res.object_list.detections[i].pose
		obj = listener.transformPose("/base_link", obj)
		obj.pose.position.z -= 0.1	#Milk z-offset
		"""
		obj = PoseStamped();
		obj.header.frame_id = "/base_link"
		obj.pose.position.x = -0.8
		obj.pose.position.y = 0
		obj.pose.position.z = 0.55
		obj.pose.orientation.x = 0
		obj.pose.orientation.y = 0
		obj.pose.orientation.z = 0
		obj.pose.orientation.w = 1


		object_id = self.getObjectID("milk");

		get_grasps_from_position = rospy.ServiceProxy('get_grasps_from_position', GetGraspsFromPosition)
		req = srs_grasping.srv.GetGraspsFromPositionRequest(object_id, obj.pose)	
		grasp_configuration = (get_grasps_from_position(req)).grasp_configuration

		for i in range(0,len(grasp_configuration)):
			pre_grasp_pose = grasp_configuration[i].pre_grasp
			grasp_pose = grasp_configuration[i].grasp

			pre_trans = grasping_functions.matrix_from_pose(pre_grasp_pose)  
			grasp_trans = grasping_functions.matrix_from_pose(grasp_pose)

			t = translation_from_matrix(pre_trans)
			q = quaternion_from_matrix(pre_trans)
			tg = translation_from_matrix(grasp_trans)
			qg = quaternion_from_matrix(grasp_trans)

			pre = PoseStamped()
			pre.header.stamp = rospy.Time.now()
			pre.header.frame_id = "/base_link"
			pre.pose.position.x = t[0]
			pre.pose.position.y = t[1]
			pre.pose.position.z = t[2] 
			pre.pose.orientation.x = q[0]
			pre.pose.orientation.y = q[1]
			pre.pose.orientation.z = q[2]
			pre.pose.orientation.w = q[3]

			g = PoseStamped()
			g.header.stamp = rospy.Time.now()
			g.header.frame_id = "/base_link"
			g.pose.position.x = tg[0]
			g.pose.position.y = tg[1]
			g.pose.position.z = tg[2]
			g.pose.orientation.x = qg[0]
			g.pose.orientation.y = qg[1]
			g.pose.orientation.z = qg[2]
			g.pose.orientation.w = qg[3]
	

			offset_x = 0#(g.pose.position.x - pre.pose.position.x)/3
			offset_y = 0#(g.pose.position.y - pre.pose.position.y)/3
			offset_z = 0#(g.pose.position.z - pre.pose.position.z)/3

			pre.pose.position.x += offset_x
			pre.pose.position.y += offset_y
			pre.pose.position.z -= offset_z
			g.pose.position.x += offset_x
			g.pose.position.y += offset_y
			g.pose.position.z -= offset_z


			sol = False
			for w in range(0,10):
				(pre_grasp_conf, error_code) = grasping_functions.callIKSolver(current_joint_configuration, pre)		
				if(error_code.val == error_code.SUCCESS):
					for k in range(0,10):
						(grasp_conf, error_code) = grasping_functions.callIKSolver(pre_grasp_conf, g)
						if(error_code.val == error_code.SUCCESS):		
							print str(i)+": IK solution found"
							sol = True
							break
					if sol:
						break


			if sol:
				print "Category: "+ grasp_configuration[i].category
				res = raw_input("Execute this grasp? (y/n): ")

				if res != "y":
					continue
				else:
					# execute grasp
					handle_say = self.sss.say(["I am grasping the object now."], False)
					handle_arm = self.sss.move("arm", [pre_grasp_conf], False)
					self.sss.move("sdh", "cylopen")
					handle_say.wait()
					handle_arm.wait()


					raw_input("Grasp...")
					handle_arm2 = self.sss.move("arm", [grasp_conf], False)
					handle_arm2.wait()

					raw_input("Catch the object")
					handle_sdh = self.sss.move("sdh", [list(grasp_configuration[i].sdh_joint_values)], False)
					handle_sdh.wait()
					"""
					# place obj on tray
					handle01 = self.sss.move("arm","grasp-to-tray",False)
					self.sss.move("tray","up")
					handle01.wait()


					self.sss.move("arm","tray")
					self.sss.move("sdh","cylopen")
					handle01 = self.sss.move("arm","tray-to-folded",False)
					self.sss.sleep(4)
					self.sss.move("sdh","cylclosed",False)
					handle01.wait()

					# deliver cup to order position
					#self.sss.move("base","order")
					self.sss.say("Here's your drink.")
					self.sss.move("torso","nod")
					"""
					print grasping_functions.sdh_tactil_sensor_result()
					return 0

			#else:
				#rospy.logerr(str(i)+": Ik grasp FAILED")

		return -1


    	def get_joint_state(self, msg):
		global current_joint_configuration
		current_joint_configuration = list(msg.desired.positions)
		rospy.spin()


	#Fake function
	def getObjectID(self, obj_name):
		if (obj_name=="milk"): 
			return 1;
		else:
			return -1;


if __name__ == "__main__":
    	rospy.init_node('grasp_test')
	SCRIPT = GraspScript()
	SCRIPT.execute()
