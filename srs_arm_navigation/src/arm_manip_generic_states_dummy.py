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
    
