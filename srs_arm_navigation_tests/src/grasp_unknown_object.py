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
from simulate_dm import *

def main():
  
  rospy.init_node('fake_dm_grasping', log_level=rospy.DEBUG)
  rospy.loginfo("Script for testing generic state for grasping of unknown object")
  
  # sm_to = move_arm_to_given_positions_assisted()
  
  sm = smach.StateMachine(outcomes=['fdm_completed','fdm_not_completed','fdm_failed','fdm_pre-empted'])

  
  with sm:
    
    smach.StateMachine.add('grasp',grasp_unknown_object_assisted(),
                           transitions={'completed':'fdm_completed',
                                        'not_completed':'fdm_not_completed',
                                        'pre-empted':'fdm_pre-empted',
                                        'failed':'fdm_failed'})
    
  rospy.loginfo('Executing state machine...') 
  output = sm.execute()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
  
