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
from srs_env_model.srv import LoadSave
import sys
import os

class LoaderSaver():
    
    def __init__(self):
        
        self.loader = rospy.ServiceProxy("/but_plane_detector/load_planes", LoadSave)
        self.saver = rospy.ServiceProxy("/but_plane_detector/save_planes", LoadSave)
        
    def load(self,fname):
        
        rospy.loginfo("Trying to load geometric map (from %s)",str(fname))
        
        available = False
    
        try:
            
            rospy.wait_for_service("/but_plane_detector/load_planes",timeout=60)
            available = True
            
        except ROSException, e:
            
           pass
       
        if not available:
           
           rospy.logerr("Service is not available")
           return
        
        try:
            
            res = self.loader(filename = fname)
            
        except Exception, e:
        
            rospy.logerr('Cannot load geometric map, error: %s',str(e))
            
        if res.all_ok:
            
            rospy.loginfo("Geometric map has been loaded.")
        
        
    def save(self,fname):
        
        rospy.loginfo("Trying to save geometric map (to %s)",str(fname))
        
        available = False
    
        try:
            
            rospy.wait_for_service("/but_plane_detector/save_planes",timeout=60)
            available = True
            
        except ROSException, e:
            
           pass
       
        if not available:
           
           rospy.logerr("Service is not available")
           return
        
        try:
    
            res = self.saver(filename = fname)
            
        except Exception, e:
        
            rospy.logerr('Cannot save the geometric map, error: %s',str(e))
            
        if res.all_ok:
            
            rospy.loginfo("The geometric map has been saved.")
        
        
        

if __name__ == '__main__':

  try:
      
    rospy.init_node('geommap_loader')
      
    l = LoaderSaver()
    
    if not rospy.has_param('~action'):
        
        rospy.logerr('Param "action" is not set')
        sys.exit()
        
    if not rospy.has_param('~file'):
        
        rospy.logerr('Param "file" is not set')
        sys.exit()


    action = str(rospy.get_param('~action'))
    file = str(rospy.get_param('~file'))
    
    if action == "load":
        
        if not os.path.exists(file):
        
            rospy.logerr("File (%s) does not exist!",file)
            sys.exit()
        
        l.load(file)
        
    elif action == "save":
    
        l.save(file)
        
    else:
        
        rospy.logerr("Unknown action. Please use 'load' or 'save'.")
    
  except rospy.ROSInterruptException:
    pass
