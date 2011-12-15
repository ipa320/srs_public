#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy
import math

import tf
from tf.transformations import euler_from_quaternion



def handle_symbol_grounding_grasp_base_pose(req):
	
	obj_x = req.obj_pose.position.x
	obj_y = req.obj_pose.position.y
	obj_rpy = tf.transformations.euler_from_quaternion([req.obj_pose.orientation.x, req.obj_pose.orientation.y, req.obj_pose.orientation.z, req.obj_pose.orientation.w])
	obj_th = obj_rpy[2]
	rospy.loginfo(obj_th)

	rb_x = req.rb_pose.x
	rb_y = req.rb_pose.y
	rb_th = req.rb_pose.theta

	gbp = Pose2D()


	#right grasp
	if req.grasp == 1:
		
		bgp_x = rb_x - 0.8*math.cos(rb_th) + 0.15*math.sin(rb_th)
		bgp_y = rb_y + 0.15*math.cos(rb_th) - 0.8*math.sin(rb_th)
		bgp_th = rb_th
		delta_x = obj_x - bgp_x
		delta_y = obj_y - bgp_y
		delta_th = obj_th - bgp_th
		gbp_x = obj_x + 0.8*math.cos(obj_th) - 0.15*math.sin(obj_th)
		gbp_y = obj_y + 0.15*math.cos(obj_th) + 0.8*math.sin(obj_th)
		gbp_th = obj_th
		if delta_x > 0.1 or delta_x < -0.15:
			reach = 0
		elif delta_y > 0.15 or delta_y < -0.1:
			reach = 0
		elif delta_th > (0.0556 * math.pi) or delta_th < (-0.0556 * math.pi):
			reach = 0
		else:
			index_x = round(delta_x / 0.025) + 6
		        index_y = round(delta_y / 0.025) + 6
		        index_th = round(delta_th / 0.0175) + 10 
		        index_x = int(index_x)
		        index_y = int(index_y)
		        index_th = int(index_th)
		        member_x = mf1_x[index_x]
		        member_y = mf1_y[index_y]
		        member_th = mf1_th[index_th]
			#Apply the fuzzy rule.
			reach = min(member_x, member_y, member_th)
			
	#front grasp
	elif grasp == 2:
				
		bgp_x = rb_x - 0.85*math.cos(rb_th) - 0.1*math.sin(rb_th)
		bgp_y = rb_y - 0.1*math.cos(rb_th) - 0.85*math.sin(rb_th)
		bgp_th = rb_th
		delta_x = obj_x - bgp_x
		delta_y = obj_y - bgp_y
		delta_th = obj_th - bgp_th
		gbp_x = obj_x + 0.85*math.cos(obj_th) + 0.1*math.sin(obj_th)
		gbp_y = obj_y + 0.1*math.cos(obj_th) - 0.85*math.sin(obj_th)
		gbp_th = obj_th
		
		if delta_x > 0.1 or delta_x < -0.15:
			reach = 0
		elif delta_y > 0.15 or delta_y < -0.1:
			reach = 0
		elif delta_th > (0.0556 * math.pi) or delta_th < (-0.0556 * math.pi):
			reach = 0
		else:
			index_x = round(delta_x / 0.025) + 6
		        index_y = round(delta_y / 0.025) + 6
		        index_th = round(delta_th / 0.0175) + 10
		        index_x = int(index_x)
		        index_y = int(index_y)
		        index_th = int(index_th)
		        member_x = mf2_x[index_x]
		        member_y = mf2_y[index_y]
		        member_th = mf2_th[index_th]
			#Apply the fuzzy rule.
			reach = min(member_x, member_y, member_th)

	#top grasp
	elif req.grasp == 3:

		bgp_x = rb_x - 0.8*math.cos(rb_th) - 0.1*math.sin(rb_th)
		bgp_y = rb_y - 0.1*math.cos(rb_th) - 0.8*math.sin(rb_th)
		delta_x = obj_x - bgp_x
		delta_y = obj_y - bgp_y
		gbp_x = obj_x + 0.8*math.cos(obj_th) + 0.1*math.sin(obj_th)
		gbp_y = obj_y + 0.1*math.cos(obj_th) - 0.8*math.sin(obj_th)
		gbp_th = math.atan((obj_y - gbp_y) / (gbp_x - obj_x))
		
		if delta_x > 0.1 or delta_x < -0.15:
			reach = 0
		elif delta_y > 0.1 or delta_y < -0.1:
			reach = 0
		else: 
			index_x = round(delta_x / 0.025) + 6
		        index_y = round(delta_y / 0.025) + 6
		        index_x = int(index_x)
		        index_y = int(index_y)
		        index_th = int(index_th)
		        member_x = mf3_x[index_x]
		        member_y = mf3_y[index_y]	   
			#Apply the fuzzy rule.
			reach = min(member_x, member_y)

	gbp.x = gbp_x
	gbp.y = gbp_y
	gbp.theta = gbp_th
	
	return reach, gbp
	



def symbol_grounding_grasp_base_pose_server():
	rospy.init_node('symbol_grounding_grasp_base_pose_server')
	s = rospy.Service('symbol_grounding_grasp_base_pose', SymbolGroundingGraspBasePose, handle_symbol_grounding_grasp_base_pose)
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


if __name__ == "__main__":
    symbol_grounding_grasp_base_pose_server()
