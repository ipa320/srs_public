#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePoseExperimental
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import math

import tf
from tf.transformations import euler_from_quaternion



def handle_symbol_grounding_grasp_base_pose_experimental(req):
	
	target_obj_x = req.target_obj_pose.position.x
	target_obj_y = req.target_obj_pose.position.y
	target_obj_rpy = tf.transformations.euler_from_quaternion([req.target_obj_pose.orientation.x, req.target_obj_pose.orientation.y, req.target_obj_pose.orientation.z, req.target_obj_pose.orientation.w])
	target_obj_th = target_obj_rpy[2]
	#rospy.loginfo(target_obj_th)

	robot_base_pose_x = req.rb_pose.x
	robot_base_pose_y = req.rb_pose.y
	robot_base_pose_th = req.rb_pose.theta

	parent_obj_x = req.parent_obj_geometry.pose.x
	parent_obj_y = req.parent_obj_geometry.pose.y
	parent_obj_th = req.parent_obj_geometry.pose.theta
	parent_obj_l = req.parent_obj_geometry.l
	parent_obj_w = req.parent_obj_geometry.w
	parent_obj_h = req.parent_obj_geometry.h

	grasp_base_pose = Pose2D()



	#right grasp

	best_grasp_pose_x = robot_base_pose_x - 0.8 * math.cos(robot_base_pose_th) + 0.15 * math.sin(robot_base_pose_th)
	best_grasp_pose_y = robot_base_pose_y - 0.8 * math.sin(robot_base_pose_th) - 0.15 * math.cos(robot_base_pose_th) 

	delta_x = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.cos(target_obj_th - robot_base_pose_th) 
	delta_y = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.sin(target_obj_th - robot_base_pose_th) 
	delta_th = robot_base_pose_th - parent_obj_th

	right_grasp_base_pose_x = target_obj_x + 0.8 * math.cos(parent_obj_th) - 0.15 * math.sin(parent_obj_th)
	right_grasp_base_pose_y = target_obj_y + 0.8 * math.sin(parent_obj_th) + 0.15 * math.cos(parent_obj_th)
	right_grasp_base_pose_th = parent_obj_th

	if delta_x > 0.1 or delta_x < -0.15:
		right_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		right_grasp_reach = 0
	elif delta_th > (10.0 / 180.0 * math.pi) or delta_th < (-10.0 / 180.0 * math.pi):
		right_grasp_reach = 0
	else:
		index_x = int(round(delta_x / 0.025 + 6))
		index_y = int(round(delta_y / 0.025 + 6))
		index_th = int(round(delta_th / (1.0 / 180.0 * math.pi) + 10)) 
		member_x = mf1_x[index_x]
		member_y = mf1_y[index_y]
		member_th = mf1_th[index_th]
		#Apply the fuzzy rule.
		right_grasp_reach = min(member_x, member_y, member_th)
		rospy.loginfo(right_grasp_reach)

		
	
	#front grasp

	best_grasp_pose_x = robot_base_pose_x - 0.85 * math.cos(robot_base_pose_th) + 0.1 * math.sin(robot_base_pose_th)
	best_grasp_pose_y = robot_base_pose_y - 0.85 * math.sin(robot_base_pose_th) - 0.1 * math.cos(robot_base_pose_th) 

	delta_x = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.cos(target_obj_th - robot_base_pose_th)
	delta_y = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.sin(target_obj_th - robot_base_pose_th)
	delta_th = robot_base_pose_th - parent_obj_th

	front_grasp_base_pose_x = target_obj_x + 0.85 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th) 
	front_grasp_base_pose_y = target_obj_y + 0.85 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th) 
	front_grasp_base_pose_th = parent_obj_th
	#rospy.loginfo([delta_x, delta_y, delta_th])

	if delta_x > 0.1 or delta_x < -0.15:
		front_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		front_grasp_reach = 0
	elif delta_th > (10.0 / 180.0 * math.pi) or delta_th < (-10.0 / 180.0 * math.pi):
		front_grasp_reach = 0
	else:
		index_x = int(round(delta_x / 0.025 + 6))
		index_y = int(round(delta_y / 0.025 + 6))
		index_th = int(round(delta_th / (1.0 / 180.0 * math.pi) + 10)) 
		member_x = mf2_x[index_x]
		member_y = mf2_y[index_y]
		member_th = mf2_th[index_th]
		#Apply the fuzzy rule.
		front_grasp_reach = min(member_x, member_y, member_th)
		rospy.loginfo(front_grasp_reach)
	


	#top grasp

	best_grasp_pose_x = robot_base_pose_x - 0.8 * math.cos(robot_base_pose_th) + 0.1 * math.sin(robot_base_pose_th)
	best_grasp_pose_y = robot_base_pose_y - 0.8 * math.sin(robot_base_pose_th) - 0.1 * math.cos(robot_base_pose_th) 

	delta_x = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.cos(target_obj_th - robot_base_pose_th)
	delta_y = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.sin(target_obj_th - robot_base_pose_th)
	delta_th = robot_base_pose_th - parent_obj_th

	top_grasp_base_pose_x = target_obj_x + 0.8 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th) 
	top_grasp_base_pose_y = target_obj_y + 0.8 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th) 
	top_grasp_base_pose_th = parent_obj_th

	if delta_x > 0.1 or delta_x < -0.15:
		top_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		top_grasp_reach = 0
	elif delta_th > (10.0 / 180.0 * math.pi) or delta_th < (-10.0 / 180.0 * math.pi):
		top_grasp_reach = 0
	else:
		index_x = int(round(delta_x / 0.025 + 6))
		index_y = int(round(delta_y / 0.025 + 6))
		index_th = int(round(delta_th / (1.0 / 180.0 * math.pi) + 10)) 
		member_x = mf3_x[index_x]
		member_y = mf3_y[index_y]
		member_th = mf3_th[index_th]
		#Apply the fuzzy rule.
		top_grasp_reach = min(member_x, member_y, member_th)
		rospy.loginfo(top_grasp_reach)



	reach = max(right_grasp_reach, front_grasp_reach, top_grasp_reach)

	grasp_base_pose_x = (right_grasp_base_pose_x + front_grasp_base_pose_x + top_grasp_base_pose_x) / 3.0
	grasp_base_pose_y = (right_grasp_base_pose_y + front_grasp_base_pose_y + top_grasp_base_pose_y) / 3.0
	grasp_base_pose_th = right_grasp_base_pose_th
	grasp_base_pose.x = grasp_base_pose_x
	grasp_base_pose.y = grasp_base_pose_y
	grasp_base_pose.theta = grasp_base_pose_th

	
	index = 0
	while index < len(req.furnitures_geometry):
		delta_x = math.sqrt(grasp_base_pose_x ** 2 + grasp_base_pose_y ** 2) * math.cos(grasp_base_pose_th - req.furnitures_geometry[index].pose.theta) - req.furnitures_geometry[index].pose.x
		delta_y = math.sqrt(grasp_base_pose_x ** 2 + grasp_base_pose_y ** 2) * math.sin(grasp_base_pose_th - req.furnitures_geometry[index].pose.theta) - req.furnitures_geometry[index].pose.y
		if (-(req.furnitures_geometry[index].w / 2.0 + 0.5) <= delta_x <= (req.furnitures_geometry[index].w / 2.0 + 0.5)) or (-(req.furnitures_geometry[index].l / 2.0 + 0.5) <= delta_y <= (req.furnitures_geometry[index].l / 2.0 + 0.5)):
			obstacle_check = "grasp base pose is blocked!"
		else:
			obstacle_check = "ready to move!"
		index += 1
		

	return obstacle_check, reach, grasp_base_pose

	



def symbol_grounding_grasp_base_pose_experimental_server():
	rospy.init_node('symbol_grounding_grasp_base_pose_experimental_server')
	s = rospy.Service('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental, handle_symbol_grounding_grasp_base_pose_experimental)
	print "Ready to receive requests."
	rospy.spin()


mf1_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf1_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125, 0]
mf1_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

mf2_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf2_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
mf2_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

mf3_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf3_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
mf3_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]


if __name__ == "__main__":
    symbol_grounding_grasp_base_pose_experimental_server()
