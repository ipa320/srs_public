#!/usr/bin/python

PKG = 'srs_states'
import roslib; roslib.load_manifest(PKG)
import rospy
import smach
import smach_ros
import sys
from geometry_msgs.msg import Pose

#import unittest

from detection_states import *

class TestStates:
  def __init__(self, *args):
    rospy.init_node('test_states')

  def test_table_object_cluster(self):
    # create a SMACH state machine
    SM = smach.StateMachine(outcomes=['overall_succeeded','overall_failed'])
    #SM.userdata.object_id = 0
    SM.userdata.target_table_pose = Pose()
    SM.userdata.target_table_pose.position.x = 0.67
    SM.userdata.target_table_pose.position.y = 1.26
    SM.userdata.target_table_pose.position.z = 0.74
    SM.userdata.target_table_pose.orientation.w = 0
    SM.userdata.target_table_pose.orientation.x = 0
    SM.userdata.target_table_pose.orientation.y = 0
    SM.userdata.target_table_pose.orientation.z = 0
    SM.userdata.target_object_pose_on_table = Pose()
    SM.userdata.target_object_pose_on_table.position.x = 0.7
    SM.userdata.target_object_pose_on_table.position.y = 1.0
    SM.userdata.target_object_pose_on_table.position.z = 0.74
    SM.userdata.target_object_pose_on_table.orientation.w = 0
    SM.userdata.target_object_pose_on_table.orientation.x = 0
    SM.userdata.target_object_pose_on_table.orientation.y = 0
    SM.userdata.target_object_pose_on_table.orientation.z = 0
    #SM.userdata.verfied_target_object_pose = Pose()

    # open the container
    with SM:
      smach.StateMachine.add('TABLE_CLUSTER', GetTableObjectCluster(),
        transitions={'succeeded':'CHECK_FREE', 'failed':'overall_failed', 'not_completed':'overall_failed', 'preempted':'overall_failed'})
      smach.StateMachine.add('CHECK_FREE', CheckPoseOnTableFree(),
        transitions={'succeeded':'overall_succeeded', 'failed':'overall_failed', 'not_completed':'overall_failed', 'preempted':'overall_failed'})
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
    test.test_table_object_cluster()
