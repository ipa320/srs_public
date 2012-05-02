#!/usr/bin/env python

# dummy functions, 
# for maintaining package stability if srs_assisted_arm_navigation is not working or failed to compile 
 

import roslib; roslib.load_manifest('srs_states')
import rospy
import smach

# estimate the best grasp position
class assisted_arm_navigation_prepare(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['object'],
            output_keys=['pose_of_the_target_object','bb_of_the_target_object'])

    def execute(self, userdata):
        userdata.pose_of_the_target_object=''
        userdata.bb_of_the_target_object=''
        return 'failed'


class move_arm_to_given_positions_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['succeeded','not_completed','failed','preempted'],
                         input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object','pose_of_the_target_object','bb_of_the_target_object'],
                         output_keys=['id_of_the_reached_position'])  
  def execute(self,userdata):      
    userdata.id_of_the_reached_position =''
    return 'failed'


class move_arm_from_a_given_position_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['succeeded','not_completed','failed','preempted'])
    
  def execute(self,userdata):
    return 'failed'
