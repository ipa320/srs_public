#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_arm_navigation')
import rospy
import smach
import smach_ros
import actionlib
from srs_arm_navigation.msg import *



class move_arm_to_given_positions_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,
                         outcomes=['completed','not_completed','failed','pre-empted'],
                         input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object',
                                     'pose_of_the_target_object','bb_of_the_target_object'],
                         output_keys=['id_of_the_reached_position'])
    
  def execute(self,userdata):
    
    rospy.loginfo('Executing state move_arm_to_given_positions_assisted')    
    
    return 'failed'
    


class move_arm_from_a_given_position_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'])
    
  def execute(self,userdata):
    
    rospy.loginfo('Executing state move_arm_from_a_given_position_assisted')
    
    return 'failed'
    
