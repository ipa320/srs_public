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
from cob_script_server.msg import ScriptAction
from cob_script_server.msg import ScriptGoal

from srs_assisted_grasping_msgs.msg import *

def main():
  rospy.init_node('grasping_action_test')
  rospy.loginfo("Node for testing reactive grasping actionlib server")
  
  
  script_client = actionlib.SimpleActionClient('/script_server',ScriptAction)
  
  rospy.loginfo('Waiting for cob script server')
  script_client.wait_for_server()
  
  script_goal = ScriptGoal()
  
  script_goal.component_name = 'sdh'
  script_goal.function_name = 'move'
  script_goal.parameter_name = 'cylopen'
  #script_goal.parameter_name = 'cyltotalopen'
  script_goal.mode = ''
  
  script_client.send_goal(script_goal)
  
  script_client.wait_for_result()
  
 
  client = actionlib.SimpleActionClient('/but_arm_manip/reactive_grasping_action',ReactiveGraspingAction)
  
  rospy.loginfo("Waiting for reactive grasping server...")
  client.wait_for_server()
  
  rospy.loginfo('Opening gripper using cob_script_server')
  goal = ReactiveGraspingGoal()
  
  #goal.target_configuration.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # home
  goal.target_configuration.data = [0.0, 0.0, 1.0472, 0.0, 1.0472, 0.0, 1.0472] # cylclosed
  #goal.target_configuration.data = [1.047,-0.262,1.047,-0.262,1.047,-0.262,1.047] # spherclosed
  goal.max_force.data = [300.0, 300.0, 300.0, 300.0, 300.0, 300.0] # 6 tactile pads
  goal.time = rospy.Duration(4.0)
  
  client.send_goal(goal)

  rospy.loginfo('Closing gripper using reactive grasping')
  client.wait_for_result()
  
  result = client.get_result()
  
  print result.actual_joint_values
  print result.actual_forces
  print result.time_to_stop
   
  
  
if __name__ == '__main__':

  try:
      main()
  except rospy.ROSInterruptException: pass
