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
from srs_interaction_primitives.srv import GetUnknownObject, AddUnknownObject, RemovePrimitive
from geometry_msgs.msg import Pose, Vector3
from srs_interaction_primitives.msg import PoseType

def main():
    
    rospy.init_node('print_bb_node')
    
    s_get = rospy.ServiceProxy("/interaction_primitives/get_unknown_object", GetUnknownObject)
    s_add = rospy.ServiceProxy("/interaction_primitives/add_unknown_object", AddUnknownObject)
    s_remove = rospy.ServiceProxy("/interaction_primitives/remove_primitive", RemovePrimitive)
    
    rospy.wait_for_service("/interaction_primitives/get_unknown_object")
    rospy.wait_for_service("/interaction_primitives/add_unknown_object")
    rospy.wait_for_service("/interaction_primitives/remove_primitive")
    
    bb_pose = Pose()
    
    bb_pose.position.x = rospy.get_param("~bb/position/x")
    bb_pose.position.y = rospy.get_param("~bb/position/y")
    bb_pose.position.z = rospy.get_param("~bb/position/z")
    
    bb_pose.orientation.x = rospy.get_param("~bb/orientation/x")
    bb_pose.orientation.y = rospy.get_param("~bb/orientation/y")
    bb_pose.orientation.z = rospy.get_param("~bb/orientation/z")
    bb_pose.orientation.w = rospy.get_param("~bb/orientation/w")
    
    lwh = Vector3()
    
    lwh.x = 2*rospy.get_param("~bb/lwh/x")
    lwh.y = 2*rospy.get_param("~bb/lwh/y")
    lwh.z = rospy.get_param("~bb/lwh/z")
    
    try:
            
      ret = s_add(frame_id= "/map",
                  name="unknown_object",
                  description="",
                  pose_type = PoseType.POSE_BASE,
                  pose = bb_pose,
                  scale = lwh,
                  disable_material=True)
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
      
    raw_input("Please press enter when finished!")
    
    ret = None
    
    try:
            
      ret = s_get(name= "unknown_object")
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e))
      
    print "bb:"
    print " position:"
    print "  x: " + str(ret.pose.position.x)
    print "  y: " + str(ret.pose.position.y)
    print "  z: " + str(ret.pose.position.z - (ret.scale.z/2.0))
    
    print " orientation:"
    print "  x: " + str(ret.pose.orientation.x)
    print "  y: " + str(ret.pose.orientation.y)
    print "  z: " + str(ret.pose.orientation.z)
    print "  w: " + str(ret.pose.orientation.w)
    
    print " lwh:"
    print "  x: " + str(ret.scale.x/2.0)
    print "  y: " + str(ret.scale.y/2.0)
    print "  z: " + str(ret.scale.z)
    
    
    try:
            
      ret = s_remove(name = "unknown_object")
            
    except Exception, e:
        
      rospy.logerr('Error on calling service: %s',str(e)) 
    
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass