#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import SymbolGroundingGraspBasePose
import rospy
import math

def handle_symbol_grounding_grasp_base_pose(req):
	
	gbp = list()
	#right grasp
	if req.grasp == 1:
		
		bgp_x = req.rb_x - 0.8*math.cos(req.rb_th) + 0.15*math.sin(req.rb_th)
		bgp_y = req.rb_y + 0.15*math.cos(req.rb_th) - 0.8*math.sin(req.rb_th)
		bgp_th = req.rb_th
		delta_x = req.obj_x - bgp_x
		delta_y = req.obj_y - bgp_y
		delta_th = req.obj_th - bgp_th
		gbp_x = req.obj_x + 0.8*math.cos(req.obj_th) - 0.15*math.sin(req.obj_th)
		gbp_y = req.obj_y + 0.15*math.cos(req.obj_th) + 0.8*math.sin(req.obj_th)
		gbp_th = req.obj_th
		if delta_x > 0.1 or delta_x < -0.15:
			reach = 0
		elif delta_y > 0.15 or delta_y < -0.1:
			reach = 0
		elif delta_th > ((10 / 180) * math.pi) or delta_th < ((-10 / 180) * math.pi):
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
	elif req.grasp == 2:
				
		bgp_x = req.rb_x - 0.85*math.cos(req.rb_th) - 0.1*math.sin(req.rb_th)
		bgp_y = req.rb_y - 0.1*math.cos(req.rb_th) - 0.85*math.sin(req.rb_th)
		bgp_th = req.rb_th
		delta_x = req.obj_x - bgp_x
		delta_y = req.obj_y - bgp_y
		delta_th = req.obj_th - bgp_th
		gbp_x = req.obj_x + 0.85*math.cos(req.obj_th) + 0.1*math.sin(req.obj_th)
		gbp_y = req.obj_y + 0.1*math.cos(req.obj_th) - 0.85*math.sin(req.obj_th)
		gbp_th = req.obj_th
		
		if delta_x > 0.1 or delta_x < -0.15:
			reach = 0
		elif delta_y > 0.15 or delta_y < -0.1:
			reach = 0
		elif delta_th > (10 / 180 * math.pi) or delta_th < (-10 / 180 * math.pi):
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

		bgp_x = req.rb_x - 0.8*math.cos(req.rb_th) - 0.1*math.sin(req.rb_th)
		bgp_y = req.rb_y - 0.1*math.cos(req.rb_th) - 0.8*math.sin(req.rb_th)
		delta_x = req.obj_x - bgp_x
		delta_y = req.obj_y - bgp_y
		gbp_x = req.obj_x + 0.8*math.cos(req.obj_th) + 0.1*math.sin(req.obj_th)
		gbp_y = req.obj_y + 0.1*math.cos(req.obj_th) - 0.8*math.sin(req.obj_th)
		gbp_th = math.atan((req.obj_y - gbp_y) / (gbp_x - req.obj_x))
		
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

	gbp.append([reach, gbp_x, gbp_y, gbp_th])
	
	return gbp
	



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
