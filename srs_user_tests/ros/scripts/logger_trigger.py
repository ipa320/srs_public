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


def main():
    
    rospy.init_node('logger_trigger_node')
    
    s_log = rospy.ServiceProxy("/logger/start", Empty)
    
    rospy.loginfo("Waiting for logger...")

    rospy.wait_for_service("/logger/start")
    
    rospy.sleep(2)

    rospy.loginfo("Starting logging")
    
    try:
            
      s_log()
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
    rospy.loginfo("Finished!")
      

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
