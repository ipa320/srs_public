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
# Author: Tomas Lokaj
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

"""
THIS IS ONLY A TESTING FILE!
"""

import roslib; roslib.load_manifest('interaction_primitives');
import rospy
from math import fabs
import sys

from cob_object_detection_msgs.srv import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import RegionOfInterest
from srs_interaction_primitives.srv import AddObject, RemovePrimitive


def detect_object(object_name):
    print "Waiting for detector... ",
    sys.stdout.flush()
    rospy.wait_for_service('object_detection/detect_object')
    print 'ok'
    
    
    print "Detecting milk...",
    sys.stdout.flush() 
    detect_object = rospy.ServiceProxy('object_detection/detect_object', DetectObjects)
    try:
        object_name = String()
        object_name.data = 'milk'
        roi = RegionOfInterest()
        
        objects = detect_object(object_name, roi)
    except:
        print "ERROR!\nexiting..."
        exit()
    
    print "detected" 
    
    return objects

def add_object_to_scene(object):
    print object
    
    print "Waiting for service... ",
    sys.stdout.flush()
    rospy.wait_for_service('interaction_primitives/add_object')
    print "ok"
    
    print "Adding object to scene...",
    sys.stdout.flush()
    add_object = rospy.ServiceProxy('interaction_primitives/add_object', AddObject)
    try:
        color = ColorRGBA()
        color.r = 1
        color.g = 1
        color.a = 1
        pose = object.pose.pose
        
        add_object(object.pose.header.frame_id, object.label, "Detected milk", pose, object.bounding_box_lwh, color, None, 'package://cob_gazebo_worlds/Media/models/milk_box.dae', True)
    except:
        print "ERROR!\nexiting..."
        exit()
        
    print "done"
 
  
if __name__ == '__main__':
    
    # remove old objects from scene
    rospy.wait_for_service('interaction_primitives/remove_primitive')
    remove_object = rospy.ServiceProxy('interaction_primitives/remove_primitive', RemovePrimitive)
    try:
        resp = remove_object("milk")
    except:
        pass
    
    objects = detect_object("milk")
    
    add_object_to_scene(objects.object_list.detections[0])
    
    
