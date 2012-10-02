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

import roslib; roslib.load_manifest('srs_assisted_grasping')
import rospy
import actionlib

from srs_assisted_grasping_msgs.msg import *

def main():
  rospy.init_node('grasping_action_test')
  rospy.loginfo("Node for testing reactive grasping actionlib server")
 
  client = actionlib.SimpleActionClient('/but_arm_manip/reactive_grasping_action',ReactiveGraspingAction)
  
  rospy.loginfo("Waiting for server...")
  client.wait_for_server()
  goal = ReactiveGraspingGoal()
  
  goal.target_configuration.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 7 joints
  goal.max_force.data = [300.0, 300.0, 300.0, 300.0, 300.0, 300.0] # 6 tactile pads
  goal.time = rospy.Duration(3)
  
  client.send_goal(goal)

  #timeout = 240.0
  rospy.loginfo("Waiting for result...")
  client.wait_for_result()
  
  result = client.get_result()
  
  print result.actual_joint_values
  print result.actual_forces
  print result.time_to_stop
   
  
  
if __name__ == '__main__':
  main()
