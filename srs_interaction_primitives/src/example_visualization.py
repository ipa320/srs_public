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
from srs_interaction_primitives.srv import AddObject, AddPlane, AddBillboard, AddBoundingBox, AddUnknownObject, RemovePrimitive, SetPreGraspPosition


if __name__ == '__main__':
    
    name = "object"
    
    # remove old objects from scene
    rospy.wait_for_service('but_interaction_primitives/remove_primitive')
    remove_object = rospy.ServiceProxy('but_interaction_primitives/remove_primitive', RemovePrimitive)
    try:
        remove_object(name)
        remove_object('narrow_door')
        remove_object('plane_couch')
        remove_object('walking_person')
    except:
        print "Not deleted"
        pass
    
    rospy.wait_for_service('but_interaction_primitives/add_object')
    add_object = rospy.ServiceProxy('but_interaction_primitives/add_object', AddObject)
    
    box_min = (-0.06, -0.095, 0, 1)
    box_max = (0.06, 0.095, 0.2, 1)
    scale = Vector3()
    scale.y = 0.06  #0.5 * (box_max[0] - box_min[0])
    scale.x = 0.095 #0.5 * (box_max[1] - box_min[1])
    scale.z = 0.2 #box_max[2]
    
    pose = Pose()
    pose.position.x = -0.0867321901425
    pose.position.y = -0.40233656082
    pose.position.z = 3.08555974535
    pose.orientation.x = -0.516554478301
    pose.orientation.y = 0.478412675203
    pose.orientation.z = 0.500833482007
    pose.orientation.w = 0.503446726529


    add_object('/head_color_camera_l_link', name, "Detected milk", pose, scale, ColorRGBA(0, 1, 1, 1), None, 'package://cob_gazebo_objects/Media/models/milk.dae', True)
    
    #rospy.wait_for_service('but_interaction_primitives/set_pregrasp_position')
    #set_pregrasp = rospy.ServiceProxy('but_interaction_primitives/set_pregrasp_position', SetPreGraspPosition)
    #set_pregrasp(name, 1, Pose(Vector3(0, 0, 0.5), Quaternion(0, 0.5, 0, 0.5)))
    #set_pregrasp(name, 2, Pose(Vector3(-0.25, 0, 0), Quaternion(0, 0, 0, 0)))
    #set_pregrasp(name, 3, Pose(Vector3(0, -0.25,0 ), Quaternion(0, 0, 0.5, 0.5)))


    rospy.wait_for_service('but_interaction_primitives/add_unknown_object')
    add_unknown_object = rospy.ServiceProxy('but_interaction_primitives/add_unknown_object', AddUnknownObject)
    add_unknown_object('map','narrow_door', 'Narrow door', Pose(Vector3(-0.6,2.2,1),Quaternion()),Vector3(1.0,0.01,2))
    
    rospy.wait_for_service('but_interaction_primitives/add_plane')
    add_plane = rospy.ServiceProxy('but_interaction_primitives/add_plane', AddPlane)
    add_plane('map','plane_couch', '', Pose(Vector3(2.45,0.5,0.4),Quaternion()),Vector3(1.3,0.7,2),ColorRGBA(0,1,1,1))
    
    rospy.wait_for_service('but_interaction_primitives/add_billboard')
    add_billboard = rospy.ServiceProxy('but_interaction_primitives/add_billboard', AddBillboard)
    add_billboard('map','walking_person', 'Walking person', 2, 0.9, Quaternion(0.5,0.5,0.5,0.5), Pose(Vector3(2.3,-1.5,0.8),Quaternion()),Vector3())
