#!/usr/bin/env python
import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import grasping_functions

class GENERATOR():

	def __init__(self):

		rospy.loginfo("Waiting for /get_model_mesh service...")
		rospy.wait_for_service('/get_model_mesh')
		rospy.loginfo("/get_model_mesh service found!")

		rospy.loginfo("Waiting for /insert_object_service...")
		rospy.wait_for_service('/insert_object_service')
		rospy.loginfo("/insert_object_service found!")


	def run(self, object_id):
		return grasping_functions.generator(object_id);


if __name__ == "__main__":
	rospy.init_node('grasp_generator')
	s = GENERATOR()
    	s.run(1)
