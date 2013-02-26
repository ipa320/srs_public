#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_user_tests')

import sys

import rospy
import rosservice
from std_msgs.msg import String

def bag_service_client_start():
    rospy.wait_for_service('/logger/start')
    try:
        rosservice.call_service('/logger/start', None)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def bag_service_client_stop():
    rospy.wait_for_service('/logger/stop')
    try:
        rosservice.call_service('/logger/stop', None)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
		
if __name__ == "__main__":
	while not rospy.is_shutdown():
		var = raw_input("0: stop, 1: start\n")
		if var == str(0):
			bag_service_client_stop()
		elif var == str(1):
			bag_service_client_start()
		else:
			print "wrong command"
