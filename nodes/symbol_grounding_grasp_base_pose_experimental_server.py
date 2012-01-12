#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePoseExperimental
from srs_symbolic_grounding.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
import rospy
import math

import tf
from tf.transformations import euler_from_quaternion




def handle_symbol_grounding_grasp_base_pose_experimental(req):


		

	rb_pose = Pose2D()
	rospy.Subscriber("robot_base_pose", Pose2D)
	#print rb_pose.x

	#membership functions
	mf1_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf1_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125, 0]
	mf1_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

	mf2_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf2_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
	mf2_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

	mf3_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
	mf3_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
	mf3_th = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]
	
	#transfrom
	target_obj_x = req.target_obj_pose.position.x
	target_obj_y = req.target_obj_pose.position.y
	target_obj_rpy = tf.transformations.euler_from_quaternion([req.target_obj_pose.orientation.x, req.target_obj_pose.orientation.y, req.target_obj_pose.orientation.z, req.target_obj_pose.orientation.w])
	target_obj_th = target_obj_rpy[2]
	#rospy.loginfo(target_obj_th)

	robot_base_pose_x = rb_pose.x
	robot_base_pose_y = rb_pose.y
	robot_base_pose_th = rb_pose.theta

	parent_obj_x = req.parent_obj_geometry.pose.position.x
	parent_obj_y = req.parent_obj_geometry.pose.position.y
	parent_obj_rpy = tf.transformations.euler_from_quaternion([req.parent_obj_geometry.pose.orientation.x, req.parent_obj_geometry.pose.orientation.y, req.parent_obj_geometry.pose.orientation.z, req.parent_obj_geometry.pose.orientation.w])
	parent_obj_th = parent_obj_rpy[2]
	parent_obj_l = req.parent_obj_geometry.l
	parent_obj_w = req.parent_obj_geometry.w
	parent_obj_h = req.parent_obj_geometry.h

	#transfrom list
	index = 0
	furniture_geometry_list = list()
	while index < len(req.furniture_geometry_list):
		furniture_geometry = FurnitureGeometry()
		furniture_geometry.pose.x = req.furniture_geometry_list[index].pose.position.x
		furniture_geometry.pose.y = req.furniture_geometry_list[index].pose.position.y
		furniture_pose_rpy = tf.transformations.euler_from_quaternion([req.furniture_geometry_list[index].pose.orientation.x, req.furniture_geometry_list[index].pose.orientation.y, req.furniture_geometry_list[index].pose.orientation.z, req.furniture_geometry_list[index].pose.orientation.w])		
		furniture_geometry.pose.theta = furniture_pose_rpy[2]
		furniture_geometry.l = req.furniture_geometry_list[index].l
		furniture_geometry.w = req.furniture_geometry_list[index].w
		furniture_geometry.h = req.furniture_geometry_list[index].h
		furniture_geometry_list.append(furniture_geometry)
		index += 1

	grasp_base_pose = Pose2D()



	#right grasp

	best_grasp_pose_x = robot_base_pose_x - 0.8 * math.cos(robot_base_pose_th) + 0.15 * math.sin(robot_base_pose_th)
	best_grasp_pose_y = robot_base_pose_y - 0.8 * math.sin(robot_base_pose_th) - 0.15 * math.cos(robot_base_pose_th) 

	delta_x = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.cos(target_obj_th - robot_base_pose_th) 
	delta_y = math.sqrt((target_obj_x - best_grasp_pose_x) ** 2 + (target_obj_y - best_grasp_pose_y) ** 2) * math.sin(target_obj_th - robot_base_pose_th) 
	delta_th = robot_base_pose_th - parent_obj_th


	if 80.0 / 180.0 * math.pi <= delta_th <= 100.0 / 180.0 * math.pi:
		delta_th = delta_th - 0.5 * math.pi
	elif 170.0 / 180.0 * math.pi <= delta_th <= 190.0 / 180.0 * math.pi:
		delta_th = delta_th - math.pi
	elif 260.0 / 180.0 * math.pi <= delta_th <= 280.0 / 180.0 * math.pi:
		delta_th = delta_th - 1.5 * math.pi
	#rospy.loginfo([delta_x, delta_y, delta_th])

	#grasp base pose list for right grasp
	right_grasp_base_pose_list = list()
	
	right_grasp_base_pose_1 = Pose2D()
	right_grasp_base_pose_1.x = target_obj_x + 0.8 * math.cos(parent_obj_th) - 0.15 * math.sin(parent_obj_th)
	right_grasp_base_pose_1.y = target_obj_y + 0.8 * math.sin(parent_obj_th) + 0.15 * math.cos(parent_obj_th)
	right_grasp_base_pose_1.theta = parent_obj_th
	right_grasp_base_pose_list.append(right_grasp_base_pose_1)

	right_grasp_base_pose_2 = Pose2D()
	right_grasp_base_pose_2.x = target_obj_x + 0.8 * math.sin(parent_obj_th) + 0.15 * math.cos(parent_obj_th)
	right_grasp_base_pose_2.y = target_obj_y - 0.8 * math.cos(parent_obj_th) + 0.15 * math.sin(parent_obj_th)
	right_grasp_base_pose_2.theta = parent_obj_th + 1.5 * math.pi
	right_grasp_base_pose_list.append(right_grasp_base_pose_2)

	right_grasp_base_pose_3 = Pose2D()
	right_grasp_base_pose_3.x = target_obj_x - 0.8 * math.cos(parent_obj_th) + 0.15 * math.sin(parent_obj_th)
	right_grasp_base_pose_3.y = target_obj_y - 0.8 * math.sin(parent_obj_th) - 0.15 * math.cos(parent_obj_th)
	right_grasp_base_pose_3.theta = parent_obj_th + math.pi
	right_grasp_base_pose_list.append(right_grasp_base_pose_3)

	right_grasp_base_pose_4 = Pose2D()
	right_grasp_base_pose_4.x = target_obj_x - 0.8 * math.sin(parent_obj_th) - 0.15 * math.cos(parent_obj_th)
	right_grasp_base_pose_4.y = target_obj_y + 0.8 * math.cos(parent_obj_th) - 0.15 * math.sin(parent_obj_th)
	right_grasp_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
	right_grasp_base_pose_list.append(right_grasp_base_pose_4)

	if delta_x > 0.1 or delta_x < -0.15:
		right_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		right_grasp_reach = 0
	elif delta_th < -10.0 / 180.0 * math.pi or delta_th > 10.0 / 180 * math.pi:
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

	if 80.0 / 180.0 * math.pi <= delta_th <= 100.0 / 180.0 * math.pi:
		delta_th = delta_th - 0.5 * math.pi
	elif 170.0 / 180.0 * math.pi <= delta_th <= 190.0 / 180.0 * math.pi:
		delta_th = delta_th - math.pi
	elif 260.0 / 180.0 * math.pi <= delta_th <= 280.0 / 180.0 * math.pi:
		delta_th = delta_th - 1.5 * math.pi
	#rospy.loginfo([delta_x, delta_y, delta_th])

	#grasp base poses for front grasp
	front_grasp_base_pose_list = list()
	
	front_grasp_base_pose_1 = Pose2D()
	front_grasp_base_pose_1.x = target_obj_x + 0.85 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th)
	front_grasp_base_pose_1.y = target_obj_y + 0.85 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th)
	front_grasp_base_pose_1.theta = parent_obj_th
	front_grasp_base_pose_list.append(front_grasp_base_pose_1)

	front_grasp_base_pose_2 = Pose2D()
	front_grasp_base_pose_2.x = target_obj_x + 0.85 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th)
	front_grasp_base_pose_2.y = target_obj_y - 0.85 * math.cos(parent_obj_th) + 0.1 * math.sin(parent_obj_th)
	front_grasp_base_pose_2.theta = parent_obj_th + 1.5 * math.pi
	front_grasp_base_pose_list.append(front_grasp_base_pose_2)

	front_grasp_base_pose_3 = Pose2D()
	front_grasp_base_pose_3.x = target_obj_x - 0.85 * math.cos(parent_obj_th) + 0.1 * math.sin(parent_obj_th)
	front_grasp_base_pose_3.y = target_obj_y - 0.85 * math.sin(parent_obj_th) - 0.1 * math.cos(parent_obj_th)
	front_grasp_base_pose_3.theta = parent_obj_th + math.pi
	front_grasp_base_pose_list.append(front_grasp_base_pose_3)

	front_grasp_base_pose_4 = Pose2D()
	front_grasp_base_pose_4.x = target_obj_x - 0.85 * math.sin(parent_obj_th) - 0.1 * math.cos(parent_obj_th)
	front_grasp_base_pose_4.y = target_obj_y + 0.85 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th)
	front_grasp_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
	front_grasp_base_pose_list.append(front_grasp_base_pose_4)

	if delta_x > 0.1 or delta_x < -0.15:
		front_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		front_grasp_reach = 0
	elif delta_th < -10.0 / 180.0 * math.pi or delta_th > 10.0 / 180 * math.pi:
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

	if 80.0 / 180.0 * math.pi <= delta_th <= 100.0 / 180.0 * math.pi:
		delta_th = delta_th - 0.5 * math.pi
	elif 170.0 / 180.0 * math.pi <= delta_th <= 190.0 / 180.0 * math.pi:
		delta_th = delta_th - math.pi
	elif 260.0 / 180.0 * math.pi <= delta_th <= 280.0 / 180.0 * math.pi:
		delta_th = delta_th - 1.5 * math.pi
	#rospy.loginfo([delta_x, delta_y, delta_th])

	#grasp base pose for top grasp
	top_grasp_base_pose_list = list()
	
	top_grasp_base_pose_1 = Pose2D()
	top_grasp_base_pose_1.x = target_obj_x + 0.8 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th)
	top_grasp_base_pose_1.y = target_obj_y + 0.8 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th)
	top_grasp_base_pose_1.theta = parent_obj_th
	top_grasp_base_pose_list.append(top_grasp_base_pose_1)

	top_grasp_base_pose_2 = Pose2D()
	top_grasp_base_pose_2.x = target_obj_x + 0.8 * math.sin(parent_obj_th) + 0.1 * math.cos(parent_obj_th)
	top_grasp_base_pose_2.y = target_obj_y - 0.8 * math.cos(parent_obj_th) + 0.1 * math.sin(parent_obj_th)
	top_grasp_base_pose_2.theta = parent_obj_th + 1.5 * math.pi
	top_grasp_base_pose_list.append(top_grasp_base_pose_2)

	top_grasp_base_pose_3 = Pose2D()
	top_grasp_base_pose_3.x = target_obj_x - 0.8 * math.cos(parent_obj_th) + 0.1 * math.sin(parent_obj_th)
	top_grasp_base_pose_3.y = target_obj_y - 0.8 * math.sin(parent_obj_th) - 0.1 * math.cos(parent_obj_th)
	top_grasp_base_pose_3.theta = parent_obj_th + math.pi
	top_grasp_base_pose_list.append(top_grasp_base_pose_3)

	top_grasp_base_pose_4 = Pose2D()
	top_grasp_base_pose_4.x = target_obj_x - 0.8 * math.sin(parent_obj_th) - 0.1 * math.cos(parent_obj_th)
	top_grasp_base_pose_4.y = target_obj_y + 0.8 * math.cos(parent_obj_th) - 0.1 * math.sin(parent_obj_th)
	top_grasp_base_pose_4.theta = parent_obj_th + 0.5 * math.pi
	top_grasp_base_pose_list.append(top_grasp_base_pose_4)
	if delta_x > 0.1 or delta_x < -0.15:
		top_grasp_reach = 0
	elif delta_y > 0.15 or delta_y < -0.1:
		top_grasp_reach = 0
	elif delta_th < -10.0 / 180.0 * math.pi or delta_th > 10.0 / 180 * math.pi:
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


	#the overall reachability will be the maximum of three grasp reachabilities.
	reach = max(right_grasp_reach, front_grasp_reach, top_grasp_reach)

	
	grasp_base_pose_list = list()
	if right_grasp_reach == reach:
		grasp_base_pose_list = right_grasp_base_pose_list
	elif front_grasp_reach == reach:
		grasp_base_pose_list = front_grasp_base_pose_list
	else:
		grasp_base_pose_list = top_grasp_base_pose_list
	#rospy.loginfo(grasp_base_pose_list)

	#obstacle check
	parent_obj_checked_grasp_base_pose_list = list()
	wall_checked_grasp_base_pose_list = list()
	obstacle_checked_grasp_base_pose_list = list()

	index_1 = 0
	while index_1 < len(grasp_base_pose_list):
		delta_x = math.sqrt((grasp_base_pose_list[index_1].x - parent_obj_x) ** 2 + (grasp_base_pose_list[index_1].y - parent_obj_y) ** 2) * math.cos(grasp_base_pose_list[index_1].theta - parent_obj_th)
		delta_y = math.sqrt((grasp_base_pose_list[index_1].x - parent_obj_x) ** 2 + (grasp_base_pose_list[index_1].y - parent_obj_y) ** 2) * math.sin(grasp_base_pose_list[index_1].theta - parent_obj_th)
		#rospy.loginfo([delta_x, delta_y])
		if (delta_x <= -(parent_obj_w / 2.0 + 0.5) or delta_x >= (parent_obj_w / 2.0 + 0.5)) or (delta_y <= -(parent_obj_l / 2.0 + 0.5) or delta_y >= (parent_obj_l / 2.0 + 0.5)):
			parent_obj_checked_grasp_base_pose_list.append(grasp_base_pose_list[index_1])
		index_1 += 1
	#rospy.loginfo(parent_obj_checked_grasp_base_pose_list)

	if not parent_obj_checked_grasp_base_pose_list:
		reach = 0
		obstacle_check = 1 #grasp base pose blocked
		grasp_base_pose.x = robot_base_pose_x
		grasp_base_pose.y = robot_base_pose_y
		grasp_base_pose.theta = robot_base_pose_th
	else:
		index_2 = 0
		while index_2 < len(parent_obj_checked_grasp_base_pose_list):
			if (-2.7 <= parent_obj_checked_grasp_base_pose_list[index_2].x <= 3.2) and (-1.7 <= parent_obj_checked_grasp_base_pose_list[index_2].y <= 1.6):
				wall_checked_grasp_base_pose_list.append(parent_obj_checked_grasp_base_pose_list[index_2])
			index_2 += 1
		
		#rospy.loginfo(wall_checked_grasp_base_pose_list)
		if not wall_checked_grasp_base_pose_list:
			reach = 0
			obstacle_check = 1 #grasp base pose blocked
			grasp_base_pose.x = robot_base_pose_x
			grasp_base_pose.y = robot_base_pose_y
			grasp_base_pose.theta = robot_base_pose_th

		else:
			index_3 = 0
			while index_3 < len(wall_checked_grasp_base_pose_list):
				index_4 = 0
				while index_4 < len(furniture_geometry_list):
					delta_x = math.sqrt((wall_checked_grasp_base_pose_list[index_3].x - furniture_geometry_list[index_4].pose.x) ** 2 + (wall_checked_grasp_base_pose_list[index_3].y - furniture_geometry_list[index_4].pose.y) ** 2) * math.cos(wall_checked_grasp_base_pose_list[index_3].theta - furniture_geometry_list[index_4].pose.theta)
					delta_y = math.sqrt((wall_checked_grasp_base_pose_list[index_3].x - furniture_geometry_list[index_4].pose.x) ** 2 + (wall_checked_grasp_base_pose_list[index_3].y - furniture_geometry_list[index_4].pose.y) ** 2) * math.sin(wall_checked_grasp_base_pose_list[index_3].theta - furniture_geometry_list[index_4].pose.theta)
					if (delta_x <= -(furniture_geometry_list[index_4].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_4].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_4].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_4].l / 2.0 + 0.5)):
						index_4 += 1
					else:
						index_3 += 1
						break
				obstacle_checked_grasp_base_pose_list.append(wall_checked_grasp_base_pose_list[index_3])
				index_3 += 1
	
	if not obstacle_checked_grasp_base_pose_list:
		reach = 0
		obstacle_check = 1 #grasp base pose blocked
		grasp_base_pose.x = robot_base_pose_x
		grasp_base_pose.y = robot_base_pose_y
		grasp_base_pose.theta = robot_base_pose_th
	else:	
		obstacle_check = 0 #ready to move
		grasp_base_pose.x = obstacle_checked_grasp_base_pose_list[0].x
		grasp_base_pose.y = obstacle_checked_grasp_base_pose_list[0].y
		grasp_base_pose.theta = obstacle_checked_grasp_base_pose_list[0].theta
	return obstacle_check, reach, grasp_base_pose

	



def symbol_grounding_grasp_base_pose_experimental_server():
	rospy.init_node('symbol_grounding_grasp_base_pose_experimental_server')
	s = rospy.Service('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental, handle_symbol_grounding_grasp_base_pose_experimental)
	print "Ready to receive requests."
	rospy.spin()




if __name__ == "__main__":
	symbol_grounding_grasp_base_pose_experimental_server()
