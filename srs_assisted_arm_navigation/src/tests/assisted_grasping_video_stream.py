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
from srs_env_model_percp.srv import EstimateRect
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from std_msgs.msg import Header

def main():

  rospy.init_node('assisted_grasping_video_stream')
  rospy.loginfo("Node for publishing video stream for assisted grasping")
  
  r = rospy.Rate(1)

  rospy.wait_for_service('/bb_estimator/estimate_rect')
  est_rec = rospy.ServiceProxy('/bb_estimator/estimate_rect', EstimateRect)
  
  mheader = Header()
  mpose = Pose()
  mscale = Vector3()
  
  mheader.frame_id = 'arm_7_link'
  mpose.position.x = 0
  mpose.position.y = 0
  mpose.position.z = 0
  mpose.orientation.x = 0
  mpose.orientation.y = 0
  mpose.orientation.z = 0
  mpose.orientation.w = 1
  
  mscale.x = 1
  mscale.y = 1
  mscale.z = 1
  
  while not rospy.is_shutdown():
    
    res = None
    
    mheader.stamp = rospy.Time.now()
    
    try:
    
      res = est_rec(header = mheader,
                    pose = mpose,
                    scale=mscale)
      
    except Exception, e:
        
        rospy.logerr('Error: %s',str(e))
        
    print res    
    
    
    r.sleep()
    
    
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
  