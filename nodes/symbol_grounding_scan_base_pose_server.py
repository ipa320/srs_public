#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import math
import tf
from tf.transformations import euler_from_quaternion



def handle_symbol_grounding_scan_base_pose(req):
	
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
	
	scan_base_pose_list = list()

	#get detection width
	rb_distance = 0.7
	robot_h = 1.4
	detection_angle = (45.0 / 180.0) * math.pi
	camera_distance = math.sqrt((robot_h - parent_obj_h) ** 2 + (rb_distance - 0.2) ** 2)
	detection_w = 2 * (camera_distance * math.tan(0.5 * detection_angle))	



	if ((parent_obj_th >= 0) & (parent_obj_th <= (45.0 / 180.0 * math.pi))) | ((parent_obj_th >= (135.0 / 180.0 * math.pi)) & (parent_obj_th <= (225.0 / 180.0 * math.pi))) | ((parent_obj_th >= (315.0 / 180.0 * math.pi)) & (parent_obj_th < 360)):

		if parent_obj_w > 1.5:

			for num in range(int(parent_obj_l + 0.99)):

				scan_base_pose = Pose2D()
				scan_base_pose.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
				scan_base_pose.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 *  detection_w - num * detection_w) * math.cos(parent_obj_th)
				scan_base_pose.theta = parent_obj_th + math.pi
				scan_base_pose_list.append(scan_base_pose)
				
			for num in range(int(parent_obj_l + 0.99)):

				scan_base_pose = Pose2D()
				scan_base_pose.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
				scan_base_pose.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
				scan_base_pose.theta = parent_obj_th
				scan_base_pose_list.append(scan_base_pose)

		elif (parent_obj_x - 1.5 - math.sqrt((parent_obj_l * 0.5)**2+(parent_obj_w * 0.5)**2) * math.cos(math.atan(parent_obj_l / parent_obj_w) - parent_obj_th)) >= -3.2:

			for num in range(int(parent_obj_l + 0.99)):

				scan_base_pose = Pose2D()
				scan_base_pose.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
				scan_base_pose.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
				scan_base_pose.theta = parent_obj_th + math.pi
				scan_base_pose_list.append(scan_base_pose)
				
		else:

			for num in range(int(parent_obj_l + 0.99)):

				scan_base_pose = Pose2D()
		 		scan_base_pose.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th)
				scan_base_pose.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th)
				scan_base_pose.theta = parent_obj_th
				scan_base_pose_list.append(scan_base_pose)
				rospy.loginfo(scan_base_pose)
				

	elif parent_obj_w > 1.5:

		for num in range(int(parent_obj_l + 0.99)):

			scan_base_pose = Pose2D()
			scan_base_pose.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th - 0.5 * math.pi) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th - 0.5*math.pi)
			scan_base_pose.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th - 0.5*math.pi)
			scan_base_pose.theta = parent_obj_th
			scan_base_pose_list.append(scan_base_pose)
				
		for num in range(int(parent_obj_l + 0.99)):

			scan_base_pose = Pose2D()
			scan_base_pose.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th - 0.5*math.pi)
			scan_base_pose.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th - 0.5*math.pi)
			scan_base_pose.theta = parent_obj_th + math.pi
			scan_base_pose_list.append(scan_base_pose)

	elif (parent_obj_y + 1.5 + math.sqrt((parent_obj_l * 0.5)**2+(parent_obj_w * 0.5)**2) * math.cos(math.atan(parent_obj_l / parent_obj_w) - (parent_obj_th - 0.5*math.pi))) <= 2.1:

		for num in range(int(parent_obj_l + 0.99)):

			scan_base_pose = Pose2D()
			scan_base_pose.x = parent_obj_x - (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th - 0.5 * math.pi) + (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th - 0.5 * math.pi)
			scan_base_pose.y = parent_obj_y + (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th - 0.5 * math.pi)
			scan_base_pose.theta = parent_obj_th
			scan_base_pose_list.append(scan_base_pose)

	else:

		for num in range(int(parent_obj_l + 0.99)):

			scan_base_pose = Pose2D()
			scan_base_pose.x = parent_obj_x + (parent_obj_w * 0.5 + rb_distance) * math.sin(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.cos(parent_obj_th - 0.5 * math.pi)
			scan_base_pose.y = parent_obj_y - (parent_obj_w * 0.5 + rb_distance) * math.cos(parent_obj_th - 0.5 * math.pi) - (0.5 * parent_obj_l - 0.5 * detection_w - num * detection_w) * math.sin(parent_obj_th - 0.5 * math.pi)
			scan_base_pose.theta = parent_obj_th + math.pi
			scan_base_pose_list.append(scan_base_pose)


	#rospy.loginfo(scan_base_pose_list)
	
	#obstacle check
	wall_checked_scan_base_pose_list = list()
	obstacle_checked_scan_base_pose_list = list()


	index = 0
	while index < len(scan_base_pose_list):
		if (-2.7 <= scan_base_pose_list[index].x <= 3.2) and (-1.7 <= scan_base_pose_list[index].y <= 1.6):
			wall_checked_scan_base_pose_list.append(scan_base_pose_list[index])
		index += 1
		
	#rospy.loginfo(wall_checked_scan_base_pose_list)
	if not wall_checked_scan_base_pose_list:
		reach = 0
		obstacle_check = 1

	else:
		index_1 = 0
		while index_1 < len(wall_checked_scan_base_pose_list):
			index_2 = 0
			while index_2 < len(furniture_geometry_list):
				delta_x = math.sqrt((wall_checked_scan_base_pose_list[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.cos(wall_checked_scan_base_pose_list[index_1].theta - furniture_geometry_list[index_2].pose.theta)
				delta_y = math.sqrt((wall_checked_scan_base_pose_list[index_1].x - furniture_geometry_list[index_2].pose.x) ** 2 + (wall_checked_scan_base_pose_list[index_1].y - furniture_geometry_list[index_2].pose.y) ** 2) * math.sin(wall_checked_scan_base_pose_list[index_1].theta - furniture_geometry_list[index_2].pose.theta)
				if (delta_x <= -(furniture_geometry_list[index_2].w / 2.0 + 0.5) or delta_x >= (furniture_geometry_list[index_2].w / 2.0 + 0.5)) or (delta_y <= -(furniture_geometry_list[index_2].l / 2.0 + 0.5) or delta_y >= (furniture_geometry_list[index_2].l / 2.0 + 0.5)):
					index_2 += 1
				else:
					index_1 += 1
					break
			obstacle_checked_scan_base_pose_list.append(wall_checked_scan_base_pose_list[index_1])
			index_1 += 1

	scan_base_pose_list = [obstacle_checked_scan_base_pose_list]
	
	return scan_base_pose_list



def symbol_grounding_scan_base_pose_server():
	rospy.init_node('symbol_grounding_scan_base_pose_server')
	s = rospy.Service('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose, handle_symbol_grounding_scan_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_scan_base_pose_server()


