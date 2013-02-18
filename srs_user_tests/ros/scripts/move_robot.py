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
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    
    rospy.init_node('move_robot_to_given_place')
    
    g_pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
    g_unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    g_set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,latch=True)
    rospy.sleep(26)
    
    rospy.wait_for_service("/gazebo/pause_physics")
    
    rospy.loginfo("Pausing physics")
    
    try:
            
      g_pause()
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
    
    pose = Pose()
    
    pose.position.x = rospy.get_param('~position_x')
    pose.position.y = rospy.get_param('~position_y')
    pose.position.z = rospy.get_param('~position_z')
    
    pose.orientation.x = rospy.get_param('~orientation_x')
    pose.orientation.y = rospy.get_param('~orientation_y')
    pose.orientation.z = rospy.get_param('~orientation_z')
    pose.orientation.w = rospy.get_param('~orientation_w')
    
    state = ModelState()
    
    state.model_name = "robot"
    state.pose = pose
    
    
    rospy.loginfo("Moving robot")
    try:
            
      ret = g_set_state(state)
      
      print ret.status_message
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
    
    
      
      
    rospy.loginfo("Unpausing physics") 
      
    try:
            
      g_unpause()
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
    
    
    
    loc = PoseWithCovarianceStamped()
    
    loc.pose.pose = pose
    loc.header.frame_id = "/map"
    #loc.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    rospy.loginfo("Adjusting localization")
    pub.publish(loc)
    pub.publish(loc)
    pub.publish(loc)
    
    rospy.sleep(1)
    

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
