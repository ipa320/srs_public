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
import roslib; roslib.load_manifest('srs_user_tests')
import rospy
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState

def main():
    
    rospy.init_node('get_robot_position')
    
    g_get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    
    rospy.wait_for_service("/gazebo/get_model_state")
    
    try:
            
      state = g_get_state(model_name="robot")
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      return
  
    print state.pose
  

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass