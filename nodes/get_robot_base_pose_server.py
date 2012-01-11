#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')

from srs_symbolic_grounding.srv import GetRobotBasePose
from srs_symbolic_grounding.msg import *
from geometry_msgs.msg import *
import rospy

import tf
from tf.transformations import euler_from_quaternion

def handle_get_robot_base_pose(req):
		
	listener = tf.TransformListener()
	listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(req.duration))
	try:
		now = rospy.Time.now()
		listener.waitForTransform("/map", "/base_link", now, rospy.Duration(req.duration))
		(trans,rot) = listener.lookupTransform("/map", "/base_link", now)
		#rospy.loginfo(trans)
	except (tf.LookupException, tf.ConnectivityException):
		print "error!"

	rb_pose = Pose2D()
	rb_pose.x = trans[0]
	rb_pose.y = trans[1]
	rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
	rb_pose.theta = rb_pose_rpy[2]
	



	return rb_pose

	



def get_robot_base_pose_server():
	rospy.init_node('get_robot_base_pose_server')
	s = rospy.Service('get_robot_base_pose', GetRobotBasePose, handle_get_robot_base_pose)
	print "Ready to receive requests."
	rospy.spin()



if __name__ == "__main__":
	get_robot_base_pose_server()

