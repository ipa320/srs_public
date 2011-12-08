#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingScanBasePose
import rospy
import math



def handle_symbol_grounding_scan_base_pose(req):

	sbp = list()

	if ((req.table_th >= 0) & (req.table_th <= (45 / 180 * math.pi))) | ((req.table_th >= (135 / 180 * math.pi)) & (req.table_th <= (225 / 180 * math.pi))) | ((req.table_th >= (315 / 180 * math.pi)) & (req.table_th <= 360)):

		if (req.table_x - 1.5 - math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - req.table_th)) >= -3.2:

			if (req.table_width > 1.5) & ((req.table_x + 1.5 + math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - req.table_th)) <= 3.5):

				for num in range(int(req.table_length + 0.99)):

					sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
					sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
					sbp.append(req.table_th + math.pi)

					sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
					sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
					sbp.append(req.table_th)

			else:

				for num in range(int(req.table_length + 0.99)):

					sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
					sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
					sbp.append(req.table_th + math.pi)

		elif (req.table_width > 1.5) & ((req.table_x - 1.5 - math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - req.table_th)) >= -3.2):

			for num in range(int(req.table_length + 0.99)):

				sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
				sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
				sbp.append(req.table_th)

				sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
				sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
				sbp.append(req.table_th + math.pi)

		else:
		
			for num in range(int(req.table_length + 0.99)):

				sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th) + (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th))
				sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th))
				sbp.append(req.table_th)

	elif (req.table_y + 1.5 + math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - (req.table_th - 90))) <= 2.1:

		if (req.table_width > 1.5) & ((req.table_y - 1.5 - math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - (req.table_th - 90))) >= -2.2):

			for num in range(int(req.table_length + 0.99)):

				sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
				sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
				sbp.append(req.table_th)

				
				sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
				sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
				sbp.append(req.table_th + math.pi)

		else:

			for num in range(int(req.table_length + 0.99)):

				sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
				sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
				sbp.append(req.table_th)


	elif (req.table_width > 1.5) & ((req.table_y + 1.5 + math.sqrt((req.table_length * 0.5)**2+(req.table_width * 0.5)**2) * math.cos(math.atan(req.table_length / req.table_width) - (req.table_th - 90))) <= 2.1):

		for num in range(int(req.table_length + 0.99)):

			sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
			sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
			sbp.append(req.table_th + math.pi)

			sbp.append(req.table_x - (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) + (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
			sbp.append(req.table_y + (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
			sbp.append(req.table_th)

	else:

		for num in range(int(req.table_length + 0.99)):

			sbp.append(req.table_x + (req.table_width * 0.5 + 0.5) * math.sin(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.cos(req.table_th - 90))
			sbp.append(req.table_y - (req.table_width * 0.5 + 0.5) * math.cos(req.table_th - 90) - (0.5 * req.table_length - 0.5 - num) * math.sin(req.table_th - 90))
			sbp.append(req.table_th + math.pi)

	sbp = [sbp]

	return sbp



def symbol_grounding_scan_base_pose_server():
	rospy.init_node('symbol_grounding_scan_base_pose_server')
	s = rospy.Service('symbol_grounding_scan_base_pose', SymbolGroundingScanBasePose, handle_symbol_grounding_scan_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
    symbol_grounding_scan_base_pose_server()


