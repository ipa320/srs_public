#!/usr/bin/python

PKG = 'srs_object_verification'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import sys
from geometry_msgs.msg import Pose

#import unittest

from generic_states import *

class TestStates:
  def __init__(self, *args):
    rospy.init_node('test_states')

  def test_object_verification(self):
    # create a SMACH state machine
    SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
    SM.userdata.object_id = 0
    SM.userdata.target_object_pose = Pose()
    SM.userdata.target_object_pose.position.x = 0.67
    SM.userdata.target_object_pose.position.y = 1.26
    SM.userdata.target_object_pose.position.z = 0.74
    SM.userdata.target_object_pose.orientation.w = 0
    SM.userdata.target_object_pose.orientation.x = 0
    SM.userdata.target_object_pose.orientation.y = 0
    SM.userdata.target_object_pose.orientation.z = 0
    SM.userdata.verfied_target_object_pose = Pose()

    # open the container
    with SM:
      smach.StateMachine.add('VERIFY', VerifyObject(),
        transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})
      #smach.StateMachine.add('UPDATE', Map360(),
      #  transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed'})
    try:
      SM.execute()
    except:
      error_message = "Unexpected error:", sys.exc_info()[0]
      #self.fail(error_message)

# main
if __name__ == '__main__':
    test = TestStates()
    test.test_object_verification()
