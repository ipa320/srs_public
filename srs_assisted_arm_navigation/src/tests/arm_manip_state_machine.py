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

import roslib; roslib.load_manifest('srs_assisted_arm_navigation')
import rospy
import actionlib

from srs_assisted_arm_navigation_msgs.msg import *
from srs_assisted_grasping_msgs.srv import GraspingAllow

def main():
    
  rospy.init_node('arm_manip_action_test')
  rospy.loginfo("Node for testing actionlib server")
 
  client = actionlib.SimpleActionClient('/but_arm_manip/manual_arm_manip_action',ManualArmManipAction)
  
  rospy.loginfo("Waiting for server...")
  client.wait_for_server()
  goal = ManualArmManipGoal()
  
  goal.allow_repeat = False
  goal.action = "Move arm to arbitrary position"
  goal.object_name = ""
  
  client.send_goal(goal)
  
  # call allow grasping service
  rospy.wait_for_service('/but_arm_manip/grasping_allow')
  
      
  grasping_allow = rospy.ServiceProxy('/but_arm_manip/grasping_allow',GraspingAllow)
  
  try:
      
    gr = grasping_allow(allow=True)
      
  except Exception, e:
      
    rospy.logerr("Error on calling service: %s",str(e))
  
  #timeout = 240.0
  rospy.loginfo("Waiting for result")
  client.wait_for_result()
  rospy.loginfo("I have result!! :-)")
  
  result = client.get_result()
  
  if result.success:
    rospy.loginfo("Success!")
    
  if result.failed:
    rospy.loginfo("Failed :-(")
    

  if result.timeout:
    rospy.loginfo("User is sleeping")
    
  # call allow grasping serv. again....
  try:
      
    gr = grasping_allow(allow=False)
      
  except Exception, e:
      
    rospy.logerr("Error on calling service: %s",str(e))
  
  rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())
  
if __name__ == '__main__':
  main()
