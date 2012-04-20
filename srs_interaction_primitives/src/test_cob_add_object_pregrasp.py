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

import roslib; roslib.load_manifest('srs_interaction_primitives');
import rospy
from math import fabs
import sys

from std_msgs.msg import *
from geometry_msgs.msg import *
from srs_interaction_primitives.srv import AddObject, RemovePrimitive, SetPreGraspPosition


if __name__ == '__main__':
    
    name = "object"
    
    # remove old objects from scene
    rospy.wait_for_service('interaction_primitives/remove_primitive')
    remove_object = rospy.ServiceProxy('interaction_primitives/remove_primitive', RemovePrimitive)
    try:
        resp = remove_object(name)
        print "Deleted"
    except:
        print "Not deleted"
        pass
    
    rospy.wait_for_service('interaction_primitives/add_object')
    #add_object = rospy.ServiceProxy('interaction_primitives/add_object', AddObject)
    #add_object('/world', name, '', 'package://cob_gazebo_objects/Media/models/milk.dae', True, ColorRGBA(), Pose(), Vector3())
    add_object = rospy.ServiceProxy('interaction_primitives/add_object', AddObject)
    scale = Vector3()
    scale.x = 0.001;
    scale.y = 0.001;
    scale.z = 0.001;
    add_object('/world', name, "Detected milk", Pose(), scale, ColorRGBA(0,1,1,1), None, 'package://cob_gazebo_objects/Media/models/milk.dae', True)
    
    rospy.wait_for_service('interaction_primitives/set_pregrasp_position')
    set_pregrasp = rospy.ServiceProxy('interaction_primitives/set_pregrasp_position', SetPreGraspPosition)
    set_pregrasp(name, 1, Pose(Vector3(0.1, 0.1, 0.1), Quaternion(0.2, 0.7, 0.6, 0.3)))
    set_pregrasp(name, 2, Pose(Vector3(0.3, 0.3, 0.1), Quaternion(0.8, -0.4, 0.9, 0.7)))
    set_pregrasp(name, 3, Pose(Vector3(-0.1, -0.1, -0.1), Quaternion(-0.1, 0, -0.3, 0)))
    set_pregrasp(name, 4, Pose(Vector3(0.1, -0.1, -0.1), Quaternion(0.1, 0, 0.3, 0)))
    set_pregrasp(name, 5, Pose(Vector3(-0.1, 0.1, -0.1), Quaternion(-0.1, 0, 0.3, 0)))
    set_pregrasp(name, 6, Pose(Vector3(-0.1, -0.1, 0.1), Quaternion(0.1, 0, -0.3, 0)))


    