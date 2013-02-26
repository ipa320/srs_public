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
from srs_env_model.srv import UseInputColor


def main():
    
    rospy.init_node('env_model_disable_color_node')
    
    s_dis_color = rospy.ServiceProxy("/but_env_model/server_use_input_color", UseInputColor)
    
    rospy.loginfo("Waiting for Environment Model...")

    rospy.wait_for_service("/but_env_model/server_use_input_color")
    
    rospy.sleep(2)

    rospy.loginfo("Disabling input color")
    
    try:
            
      s_dis_color(use_color = False)
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
    rospy.loginfo("Finished!")
      

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
