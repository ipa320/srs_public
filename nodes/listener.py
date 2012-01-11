#!/usr/bin/env python  
import roslib
roslib.load_manifest('srs_symbolic_grounding')
import rospy
import tf


def listener():
	rospy.init_node('listener', anonymous=True)

	listener = tf.TransformListener()
	listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
	while not rospy.is_shutdown():
		try:
			now = rospy.Time.now()
			listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
			(trans,rot) = listener.lookupTransform("/map", "/base_link", now)
		except (tf.LookupException, tf.ConnectivityException):
			continue

		rospy.loginfo(trans)

	
if __name__ == '__main__':
	listener()
