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
	
	table_x = req.furniture_geometry.pose.x
	table_y = req.furniture_geometry.pose.y
	table_th = req.furniture_geometry.pose.theta
	table_length = req.furniture_geometry.l
	table_width = req.furniture_geometry.w
	table_height = req.furniture_geometry.h
	sbps = list()


	if ((table_th >= 0) & (table_th <= (45.0 / 180.0 * math.pi))) | ((table_th >= (135.0 / 180.0 * math.pi)) & (table_th <= (225.0 / 180.0 * math.pi))) | ((table_th >= (315.0 / 180.0 * math.pi)) & (table_th <= 360)):

		if (table_x - 1.5 - math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - table_th)) >= -3.2:

			if (table_width > 1.5) & ((table_x + 1.5 + math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - table_th)) <= 3.5):

				for num in range(int(table_length + 0.99)):

					sbp = Pose2D()
					sbp.x = table_x - (table_width * 0.5 + 0.5) * math.cos(table_th) - (0.5 * table_length - 0.5 - num) * math.sin(table_th)
					sbp.y = table_y - (table_width * 0.5 + 0.5) * math.sin(table_th) + (0.5 * table_length - 0.5 - num) * math.cos(table_th)
					sbp.theta = table_th + math.pi
					sbps.append(sbp)

				for num in range(int(table_length + 0.99)):

					sbp = Pose2D()
					sbp.x = table_x + (table_width * 0.5 + 0.5) * math.cos(table_th) + (0.5 * table_length - 0.5 - num) * math.sin(table_th)
					sbp.y = table_y + (table_width * 0.5 + 0.5) * math.sin(table_th) - (0.5 * table_length - 0.5 - num) * math.cos(table_th)
					sbp.theta = table_th
					sbps.append(sbp)

			else:

				for num in range(int(table_length + 0.99)):

					sbp = Pose2D()
					sbp.x = table_x - (table_width * 0.5 + 0.5) * math.cos(table_th) - (0.5 * table_length - 0.5 - num) * math.sin(table_th)
					sbp.y = table_y - (table_width * 0.5 + 0.5) * math.sin(table_th) + (0.5 * table_length - 0.5 - num) * math.cos(table_th)
					sbp.theta = table_th + math.pi
					sbps.append(sbp)


		elif (table_width > 1.5) & ((table_x - 1.5 - math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - table_th)) >= -3.2):

			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x + (table_width * 0.5 + 0.5) * math.cos(table_th) + (0.5 * table_length - 0.5 - num) * math.sin(table_th)
				sbp.y = table_y + (table_width * 0.5 + 0.5) * math.sin(table_th) - (0.5 * table_length - 0.5 - num) * math.cos(table_th)
				sbp.theta = table_th
				sbps.append(sbp)

			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x - (table_width * 0.5 + 0.5) * math.cos(table_th) - (0.5 * table_length - 0.5 - num) * math.sin(table_th)
				sbp.y = table_y - (table_width * 0.5 + 0.5) * math.sin(table_th) + (0.5 * table_length - 0.5 - num) * math.cos(table_th)
				sbp.theta = table_th + math.pi
				sbps.append(sbp)
				
		else:
		
			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x + (table_width * 0.5 + 0.5) * math.cos(table_th) + (0.5 * table_length - 0.5 - num) * math.sin(table_th)
				sbp.y = table_y + (table_width * 0.5 + 0.5) * math.sin(table_th) - (0.5 * table_length - 0.5 - num) * math.cos(table_th)
				sbp.theta = table_th
				sbps.append(sbp)

	elif (table_y + 1.5 + math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - (table_th - 0.5*math.pi))) <= 2.1:

		if (table_width > 1.5) & ((table_y - 1.5 - math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - (table_th - 0.5*math.pi))) >= -2.2):

			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x - (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) + (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
				sbp.y = table_y + (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
				sbp.theta = table_th
				sbps.append(sbp)
				
			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x + (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
				sbp.y = table_y - (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
				sbp.theta = table_th + math.pi
				sbps.append(sbp)

		else:

			for num in range(int(table_length + 0.99)):

				sbp = Pose2D()
				sbp.x = table_x - (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) + (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
				sbp.y = table_y + (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
				sbp.theta = table_th
				sbps.append(sbp)


	elif (table_width > 1.5) & ((table_y + 1.5 + math.sqrt((table_length * 0.5)**2+(table_width * 0.5)**2) * math.cos(math.atan(table_length / table_width) - (table_th - 0.5*math.pi))) <= 2.1):

		for num in range(int(table_length + 0.99)):

			sbp = Pose2D()
			sbp.x = table_x + (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
			sbp.y = table_y - (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
			sbp.theta = table_th + math.pi
			sbps.append(sbp)

		for num in range(int(table_length + 0.99)):

			sbp = Pose2D()
			sbp.x = table_x - (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) + (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
			sbp.y = table_y + (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
			sbp.theta = table_th
			sbps.append(sbp)

	else:

		for num in range(int(table_length + 0.99)):

			sbp = Pose2D()
			sbp.x = table_x + (table_width * 0.5 + 0.5) * math.sin(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.cos(table_th - 0.5*math.pi)
			sbp.y = table_y - (table_width * 0.5 + 0.5) * math.cos(table_th - 0.5*math.pi) - (0.5 * table_length - 0.5 - num) * math.sin(table_th - 0.5*math.pi)
			sbp.theta = table_th + math.pi
			sbps.append(sbp)

	sbps = [sbps]
	
	return sbps



def symbol_grounding_scan_base_pose_server():
	rospy.init_node('symbol_grounding_scan_base_pose_server')
	s = rospy.Service('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose, handle_symbol_grounding_scan_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_scan_base_pose_server()


