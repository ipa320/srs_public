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

import roslib; roslib.load_manifest('srs_arm_navigation_tests')
import rospy


from srs_assisted_arm_navigation_msgs.msg import *
from srs_interaction_primitives.srv import RemovePrimitive, SetAllowObjectInteraction

class common_helper_methods():
    
   def __init__(self):
        
       but_gui_ns = '/interaction_primitives'
       self.s_remove_object = but_gui_ns + '/remove_primitive'
       self.s_allow_interaction = but_gui_ns + '/set_allow_object_interaction'
    
   def wait_for_srv(self,srv):
       
    available = False
      
    rospy.loginfo('Waiting for %s service',srv)
    
    try:
        
        rospy.wait_for_service(srv,timeout=60)
        available = True
        
    except ROSException, e:
        
        rospy.logerr('Cannot set interaction mode for object %s, error: %s',object_name,str(e))  
        
    
    if not available:
        
        rospy.logerr('Service %s is not available! Error: ',srv,e)
    
    return available
        
     
    
   def wait_for_topic(self,topic,topic_type):
    
    available = False
    
    rospy.loginfo('Waiting for %s topic',topic)
    
    try:
    
        rospy.wait_for_message(topic, topic_type, timeout=60)
        
    except ROSException, e:
        
        rospy.logerr('Topic %s is not available! Error: %s',topic,e)

    
    
   def remove_im(self,object_name):

    remove_object = rospy.ServiceProxy(self.s_remove_object, RemovePrimitive)
    
    removed = False
    
    try:
    
      remove_object(name=object_name)
      removed = True
      
    except Exception, e:
      
      rospy.logerr('Cannot remove IM object from the scene, error: %s',str(e))
      
    return removed

   def set_interaction(self,object_name,allow_int):
      
       int_allow_srv =  rospy.ServiceProxy(self.s_allow_interaction, SetAllowObjectInteraction)
       
       try:
           
           res = int_allow_srv(name = object_name,
                               allow = allow_int)
        
        
       except Exception, e:
        
        rospy.logerr('Cannot set interaction mode for object %s, error: %s',object_name,str(e))  

