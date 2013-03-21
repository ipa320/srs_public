#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('srs_arm_navigation_tests')
import rospy
import smach
import smach_ros
from arm_manip_generic_states import *
from assisted_grasping_generic_states import *
#from simulate_dm import *
#import actionlib
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
#from geometry_msgs.msg import PoseStamped
#from shared_state_information import *

def main():
  
  rospy.init_node('fake_dm_grasping', log_level=rospy.DEBUG)
  rospy.loginfo("Script for testing generic state for grasping of unknown object")
  
  # sm_to = move_arm_to_given_positions_assisted()
  
  sm = smach.StateMachine(input_keys=['object_name','object_description','object_bb_pose','object_bb'], outcomes=['fdm_completed','fdm_not_completed','fdm_failed','fdm_pre-empted'])
  sm.userdata.object_name='unknown_object'
  sm.userdata.object_description = 'Unknown object to grasp'
  
  sm.userdata.object_bb_pose = Pose()
  sm.userdata.object_bb_pose.position.x = rospy.get_param('~bb/position/x')
  sm.userdata.object_bb_pose.position.y = rospy.get_param('~bb/position/y')
  sm.userdata.object_bb_pose.position.z = rospy.get_param('~bb/position/z')
  
  sm.userdata.object_bb_pose.orientation.x = rospy.get_param('~bb/orientation/x')
  sm.userdata.object_bb_pose.orientation.y = rospy.get_param('~bb/orientation/y')
  sm.userdata.object_bb_pose.orientation.z = rospy.get_param('~bb/orientation/z')
  sm.userdata.object_bb_pose.orientation.w = rospy.get_param('~bb/orientation/w')
  
  sm.userdata.object_bb = Vector3()
  sm.userdata.object_bb.x = 2*rospy.get_param('~bb/lwh/x')
  sm.userdata.object_bb.y = 2*rospy.get_param('~bb/lwh/y')
  sm.userdata.object_bb.z = rospy.get_param('~bb/lwh/z') 
  
  with sm:
    
    smach.StateMachine.add('grasp',grasp_unknown_object_assisted(),
                           transitions={'completed':'fdm_completed',
                                        'not_completed':'fdm_not_completed',
                                        'pre-empted':'fdm_pre-empted',
                                        'failed':'fdm_failed',
                                        'repeat_detection' : 'fdm_failed'})
    
  rospy.loginfo('Executing state machine...') 
  output = sm.execute()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
  
