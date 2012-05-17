#!/usr/bin/python

PKG = 'srs_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
#import unittest

from mapping_states import *
from detection_states import *

class TestStates:
	def __init__(self, *args):
		rospy.init_node('test_states')

	def test_update_map(self):
		# create a SMACH state machine
		SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
		#SM.userdata.pose = "home"
		SM.userdata.angle_range = 0.6
		#SM.userdata.scan_pose = [-1.3, -1.0, 3.14]
                SM.userdata.scan_pose = [0.0, 0.0, 0.0]
                SM.userdata.object_name ='IkeaShelfMilan' 

		# open the container
		with SM:
			smach.StateMachine.add('APPROACH', ApproachScanPose(),
				transitions={'succeeded':'UPDATE', 'failed':'overall_failed'})
			smach.StateMachine.add('UPDATE', UpdateEnvMap(),
				transitions={'succeeded':'VERIFY', 'failed':'overall_failed', 'preempted':'overall_failed'})
                        smach.StateMachine.add('VERIFY', VerifyObjectByName(),
                                transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed', 'not_completed':'overall_failed', 'preempted':'overall_failed'})
			#smach.StateMachine.add('UPDATE', Map360(),
			#	transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})
		try:
			SM.execute()
		except:
			error_message = "Unexpected error:", sys.exc_info()[0]
                        print error_message
			#self.fail(error_message)

# main
if __name__ == '__main__':
    test = TestStates()
    test.test_update_map()
