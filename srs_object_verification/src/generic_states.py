#!/usr/bin/python
#################################################################
##\file
#
# \note
#   Copyright (c) 2010 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: srs
# \note
#   ROS stack name: srs
# \note
#   ROS package name: srs_object_verification
#
# \author
#   Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
#
# \date Date of creation: Jan 2012
#
# \brief
#   Implements generic states which can be used in multiple scenarios.
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib
roslib.load_manifest('srs_object_verification')
import rospy
import smach
import smach_ros
import actionlib
import operator

from struct import *

from eval_objects import *
from cob_3d_mapping_msgs.msg import *


class VerifyObject(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['object_id','target_object_pose'],
      output_keys=['verfified_target_object_pose'])
    self.eo = EvalObjects()
    #self.client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)

  def execute(self, userdata):
    #object_to_search =self.eo.semantics_db_get_object_info(userdata.object_id)
    #print "Searching for object at " + str(object_to_search.objectPose.position.x) + ", " + str(object_to_search.objectPose.position.y)
    object_list_map = self.eo.map_list_objects(1)#object_to_search.classID)
    #if object_to_search.classID == 1: #table
    verfied_table = self.eo.verify_table(userdata.target_object_pose, object_list_map)
    if verified_table:
        userdata.verfified_target_object_pose = verified_table.params[4:7]
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " found at " + str(verified_table.params[4]) + "," + str(verified_table.params[5])
        return 'succeeded'
    else:
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " not found"
        return 'not_completed'
    #else:
    #  print 'Object class not supported'
    #  return 'failed'


class CheckPositionOnTable(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['target_object_pose'],
      output_keys=[])
    self.eo = EvalObjects()
    self.client = actionlib.SimpleActionClient('table_object_cluster', TableObjectClusterAction)

  def execute(self, userdata):
    #object_to_search =self.eo.semantics_db_get_object_info(userdata.object_id)
    #print "Searching for object at " + str(object_to_search.objectPose.position.x) + ", " + str(object_to_search.objectPose.position.y)
    object_list_map = self.eo.map_list_objects(1)#object_to_search.classID)
    #if object_to_search.classID == 1: #table
    verfied_table = self.eo.verify_table(userdata.target_object_pose, object_list_map)
    #verified_table = object_list_map.objects.shapes[2]
    if verified_table:
        hull = verified_table.points[0]
        #print verified_table.params
        #print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " found at " + str(verified_table.params[4]) + "," + str(verified_table.params[5])
    else:
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " not found"
        return 'not_completed'
    goal = TableObjectClusterGoal(hull)
    if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
      rospy.logerr('server not available')
      return 'failed'
    self.client.send_goal(goal)
    if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
      return 'failed'
    bbs = self.client.get_result().bounding_boxes
    for bb in bbs:
      pt1 = []
      for i in range(0,3):
        pt1.append(unpack("f",bb.data[4*i:4*i+4])[0])
      pt2 = []
      for i in range(4,7):
        pt2.append(unpack("f",bb.data[4*i:4*i+4])[0])
      x = userdata.target_object_pose.position.x
      y = userdata.target_object_pose.position.y
      if x>pt1[0] and x<pt2[0] and y>pt1[1] and y<pt2[1]:
        print "target position occupied"
      else:
        print "target position free"
    return 'succeeded'
