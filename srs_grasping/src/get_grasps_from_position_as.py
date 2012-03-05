#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import time
import actionlib
import scipy

import grasping_functions
from srs_grasping.msg import *
from srs_msgs.msg import *
from geometry_msgs.msg import *
from kinematics_msgs.srv import *
from pr2_controllers_msgs.msg import *
from tf.transformations import *


class get_grasps_from_position_server():

	def __init__(self):

		print "---------------------------------------------------------------------------";
		rospy.loginfo("Waiting /arm_kinematics/get_ik service...");
		rospy.wait_for_service('/arm_kinematics/get_ik')
		rospy.loginfo("/arm_kinematics/get_ik is ready.");

		rospy.loginfo("Waiting /get_grasp_configurations...")
		self.client = actionlib.SimpleActionClient('/get_grasp_configurations', GraspCAction);
		self.client.wait_for_server();
		rospy.loginfo("/get_grasp_configurations is ready.")

		self.ns_global_prefix = "/get_grasps_from_position"
		self.get_grasps_from_position_server = actionlib.SimpleActionServer(self.ns_global_prefix, GraspFAction, self.execute_cb, True)
		rospy.loginfo("/get_grasps_from_position is ready.")
		self.get_grasps_from_position_server.start()
		print "---------------------------------------------------------------------------";
	
	def execute_cb(self, server_goal):
		x = time.time()
		rospy.loginfo("/get_grasps_from_position_server has been called...");

		obj_id = server_goal.object_id;
		obj = server_goal.object_pose;



		goal = GraspCGoal();
		goal.object_id = obj_id;
		self.client.send_goal(goal);
		self.client.wait_for_result(rospy.Duration.from_sec(5.0));
		grasp_configuration = (self.client.get_result()).grasp_configuration;

		#current_joint_configuration
		rospy.loginfo("Waiting /arm_controller/state...");
		sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state);
		while sub.get_num_connections() == 0:
			time.sleep(0.3);
			continue;
		rospy.loginfo("/arm_controller/state has finished.");

		rotacion = grasping_functions.rotation_matrix(obj);

		server_result = GraspFActionResult().result
		server_result.feasible_grasp_available = False;
		server_result.grasp_configuration = [];

		for i in range(0,len(grasp_configuration)):
			pre_trans = rotacion * grasping_functions.matrix_from_pose(grasp_configuration[i].pre_grasp.pose);
			grasp_trans = rotacion *  grasping_functions.matrix_from_pose(grasp_configuration[i].grasp.pose);


			t = translation_from_matrix(pre_trans);
			q = quaternion_from_matrix(pre_trans);
			tg = translation_from_matrix(grasp_trans);
			qg = quaternion_from_matrix(grasp_trans);
	
			
			pre = PoseStamped();
			pre.header.stamp = rospy.Time.now();
			pre.header.frame_id = "/base_link";
			pre.pose.position.x = t[0];
			pre.pose.position.y = t[1];
			pre.pose.position.z = t[2];
			pre.pose.orientation.x = q[0];
			pre.pose.orientation.y = q[1];
			pre.pose.orientation.z = q[2];
			pre.pose.orientation.w = q[3];

			g = PoseStamped();
			g.header.stamp = rospy.Time.now();
			g.header.frame_id = "/base_link";
			g.pose.position.x = tg[0];
			g.pose.position.y = tg[1];
			g.pose.position.z = tg[2];
			g.pose.orientation.x = qg[0];
			g.pose.orientation.y = qg[1];
			g.pose.orientation.z = qg[2];
			g.pose.orientation.w = qg[3];

			
			offset_x = 0#(g.pose.position.x - pre.pose.position.x)/3
			offset_y = 0#(g.pose.position.y - pre.pose.position.y)/3
			offset_z = 0#(g.pose.position.z - pre.pose.position.z)/3


			pre.pose.position.x += offset_x
			pre.pose.position.y += offset_y
			pre.pose.position.z -= offset_z
			g.pose.position.x += offset_x
			g.pose.position.y += offset_y
			g.pose.position.z -= offset_z
			


			sol = False;
			for w in range(0,5):
				(pre_grasp_conf, error_code) = grasping_functions.callIKSolver(current_joint_configuration, pre);		
				if(error_code.val == error_code.SUCCESS):
					for k in range(0,5):
						(grasp_conf, error_code) = grasping_functions.callIKSolver(pre_grasp_conf, g);
						if(error_code.val == error_code.SUCCESS):
							print i
							new_valid_grasp = GraspSubConfiguration();
							new_valid_grasp.sdh_joint_values = grasp_configuration[i].sdh_joint_values;
							new_valid_grasp.grasp = g.pose;
							new_valid_grasp.pre_grasp = pre.pose;
							new_valid_grasp.category = grasping_functions.get_grasp_category(pre.pose.position, g.pose.position);

							server_result.feasible_grasp_available = True;
							server_result.grasp_configuration.append(new_valid_grasp);

							sol = True;
							break;
					if sol:
						break;
			#for
		#while

		rospy.loginfo(str(len(server_result.grasp_configuration))+" valid grasps for this pose.");	
		rospy.loginfo("/get_grasps_from_position call has finished.");
		print "Time employed: " + str(time.time() - x);
		print "---------------------------------------";
		self.get_grasps_from_position_server.set_succeeded(server_result)


	def get_joint_state(self, msg):
		global current_joint_configuration;
		current_joint_configuration = list(msg.desired.positions);
		rospy.spin();


	def get_grasps_from_position_server(self):
		s = rospy.Service('/get_grasps_from_position', GetGraspsFromPosition, self.get_grasps_from_position);



## Main routine for running the grasp server
if __name__ == '__main__':
	rospy.init_node('get_grasps_from_position_server')
	SCRIPT = get_grasps_from_position_server()
	rospy.spin()
