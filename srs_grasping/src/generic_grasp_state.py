#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy

from simple_script_server import *
sss = simple_script_server()

class grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['sdh_joint_values'])
	
    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP')

	sss.move("sdh", [userdata.sdh_joint_values])

	#TODO: READ SDH SENSORS -----
	successful_grasp = True;
	# ---------------------------

	if successful_grasp:
		return 'succeeded'
	else:
		return 'failed'
