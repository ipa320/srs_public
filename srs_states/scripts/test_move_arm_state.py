#!/usr/bin/python

PKG = 'srs_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import sys
from geometry_msgs.msg import Pose

#import unittest

from move_arm_states import *

class TestStates:
  def __init__(self, *args):
    rospy.init_node('test_states')

  def test_object_verification(self):
    # create a SMACH state machine
    SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
    pose1 = PoseStamped()
    pose1.header.frame_id="sdh_palm_link"
    pose1.pose.position.x = 0
    pose1.pose.position.y = 0
    pose1.pose.position.z = 0.1
    pose1.pose.orientation.w = 1
    pose1.pose.orientation.x = 0
    pose1.pose.orientation.y = 0
    pose1.pose.orientation.z = 0
    SM.userdata.poses = [pose1]

    # open the container
    with SM:
      smach.StateMachine.add('MOVE_ARM_PLANNED', move_arm_planned(),
        transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed', 'not_completed':'overall_failed', 'preempted':'overall_failed'})
    try:
      SM.execute()
    except:
      error_message = "Unexpected error:", sys.exc_info()[0]
      #self.fail(error_message)

# main
if __name__ == '__main__':
    test = TestStates()
    test.test_object_verification()
