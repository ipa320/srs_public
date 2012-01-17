#!/usr/bin/python
import roslib
roslib.load_manifest('srs_grasping')

import actionlib
import time
import rospy
import tf
import math
import random
import scipy

from geometry_msgs.msg import *
from srs_grasping.msg import *
from kinematics_msgs.srv import *

from simple_script_server import *


from numpy import matrix

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from srs_object_database.msg import *
from srs_object_database.srv import *




class GraspScript(script):
		
	def __init__(self):

		# initialize components (not needed for simulation)
		self.sss = simple_script_server()
		
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
		
	
		self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
		self.grasp_client = actionlib.SimpleActionClient('/grasp_server', GraspAction)


	def execute(self):

		listener = tf.TransformListener(True, rospy.Duration(10.0))
		rospy.sleep(2)
		
		# prepare for grasping
		self.sss.move("base","kitchen")
		self.sss.move("arm","pregrasp")
		handle_sdh = self.sss.move("sdh","cylopen",False)
		handle_sdh.wait()
		


		#current_joint_configuration
		sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.get_joint_state)
		while sub.get_num_connections() == 0:
			time.sleep(0.3)
			continue

		
		#Detect
		self.srv_name_object_detection = '/object_detection/detect_object'
		detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
		req = DetectObjectsRequest()
		req.object_name.data = "milk"
		res = detector_service(req)

		for i in range(0,len(res.object_list.detections)):
			print str(i)+": "+res.object_list.detections[i].label
		
		
		index = -1
		while (index < 0):
			index = int(raw_input("Select object to grasp: "))
		
		obj = listener.transformPose("/base_link", res.object_list.detections[index].pose)


		while not self.grasp_client.wait_for_server(rospy.Duration(1.0)):
        		rospy.logerr('Waiting for /grasp_server...')
		rospy.loginfo('/grasp_server found')

		object_id = 1 #getID("milk")
		pose_id = raw_input("pose_id: ")

		goal = GraspGoal(object_id=object_id, pose_id=pose_id)
		self.grasp_client.send_goal(goal)
		self.grasp_client.wait_for_result()
		grasp_configuration = self.grasp_client.get_result().grasp_configuration



		for i in range(0,len(grasp_configuration)):

			pre_grasp_pose = grasp_configuration[i].pre_grasp
			grasp_pose = grasp_configuration[i].palm_pose

			# TODO:  --------------------------------
			e = euler_from_quaternion([obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w],axes='sxzy')
			rotacion =  euler_matrix(e[0],e[1],-e[2], axes='sxyz')
			rotacion[0,3] = obj.pose.position.x
			rotacion[1,3] = obj.pose.position.y
			rotacion[2,3] = obj.pose.position.z
			# -----------------------------------------

			pre_trans = rotacion * self.pose_to_mat(pre_grasp_pose)  
			grasp_trans = self.pose_to_mat(obj) *  self.pose_to_mat(grasp_pose)

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

	
			offset_x = 0 #(g.pose.position.x - pre.pose.position.x)/2
			offset_y = 0 #(g.pose.position.y - pre.pose.position.y)/2
			offset_z = 0 #(g.pose.position.z - pre.pose.position.z)/2
	

			pre.pose.position.x += offset_x
			pre.pose.position.y += offset_y
			pre.pose.position.z += offset_z
			g.pose.position.x += offset_x
			g.pose.position.y += offset_y
			g.pose.position.z += offset_z


			sol = False
			for w in range(0,10):
				(pre_grasp_conf, error_code) = self.callIKSolver(current_joint_configuration, pre)		
				if(error_code.val == error_code.SUCCESS):
					for k in range(0,10):
						(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, g)
						if(error_code.val == error_code.SUCCESS):		
							print str(i)+": IK solution found"
							sol = True
							break
					if sol:
						break


			if sol:
				res = raw_input("Execute this grasp? (y/n): ")

				if res != "y":
					continue
				else:
					# execute grasp
					self.sss.say(["I am grasping the object now."], False)
					handle_arm = self.sss.move("arm", [pre_grasp_conf], False)
					
					self.sss.move("sdh", "cylopen")
					handle_arm.wait()
					self.sss.move("arm", [grasp_conf])
					self.sss.move("sdh", [list(grasp_configuration[i].sconfiguration.points[0].positions)])
					
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
					
					return 0

			else:
				rospy.logerr(str(i)+": Ik grasp FAILED")



		return -1


	############################################################################################################

	def callIKSolver(self, current_pose, goal_pose):

		req = GetPositionIKRequest()
		req.ik_request.ik_link_name = "sdh_palm_link"
		req.ik_request.ik_seed_state.joint_state.position = current_pose
		req.ik_request.pose_stamped = goal_pose
		resp = self.iks(req)
		result = []
		for o in resp.solution.joint_state.position:
			result.append(o)
		return (result, resp.error_code)


    	def pose_to_mat(self, p): 

 	        quat = [p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] 
 	        pos = scipy.matrix([p.pose.position.x, p.pose.position.y, p.pose.position.z]).T 
 	        mat = scipy.matrix(quaternion_matrix(quat)) 
 	        mat[0:3, 3] = pos 

 	        return mat 


    	def get_joint_state(self, msg):
		global current_joint_configuration
		current_joint_configuration = list(msg.desired.positions)
		rospy.spin()


if __name__ == "__main__":
    	rospy.init_node('grasp_test')
	SCRIPT = GraspScript()
	SCRIPT.execute()
