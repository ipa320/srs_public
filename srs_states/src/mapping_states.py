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
#   Project name: care-o-bot
# \note
#   ROS stack name: cob_scenarios
# \note
#   ROS package name: cob_generic_states
#
# \author
#   Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
#
# \date Date of creation: Aug 2011
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
roslib.load_manifest('srs_states')
import rospy
import smach
import smach_ros
import actionlib
import operator

from cob_3d_mapping_msgs.msg import *
from cob_srvs.srv import Trigger

from shared_state_information import *

from simple_script_server import *
sss = simple_script_server()

## Initialize state
#
# This state will initialize all hardware drivers.
class UpdateEnvMap(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed', 'preempted'],
			input_keys=['angle_range']) #good angle value: 0.4
		self.client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)

	def execute(self, userdata):
		#scan_position = [-1.3, -1.0, 3.14]
		#sss.move("torso","home")
		#sss.move("head","front")
		#sss.move("tray","down")
		#sss.move("base",scan_position)
		rospy.wait_for_service('registration/reset',10)
		try:
			reset_registration = rospy.ServiceProxy('registration/reset', Trigger)
			resp1 = reset_registration()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		rospy.wait_for_service('point_map/clear_map',10)
		try:
			clear_point_map = rospy.ServiceProxy('point_map/clear_map', Trigger)
			resp1 = clear_point_map()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		rospy.wait_for_service('geometry_map/clear_map',10)
		try:
			clear_geom_map = rospy.ServiceProxy('geometry_map/clear_map', Trigger)
			resp1 = clear_geom_map()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		goal = TriggerMappingGoal()
		goal.start = True
		if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
			rospy.logerr('server not available')
			return 'failed'
		self.client.send_goal(goal)
		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
			return 'failed'
                sss.sleep(5.0)
		angle_start = -userdata.angle_range/2
		angle_stop = userdata.angle_range/2
		sss.move("torso",[[-0.2,angle_start,-0.1]])
		sss.move("torso",[[-0.2,angle_stop,-0.1]])
		sss.move("torso",[[0,angle_stop,0]])
		sss.move("torso",[[0,angle_start,0]])
		goal.start = False
		self.client.send_goal(goal)
		self.client.wait_for_result(rospy.Duration.from_sec(5.0))
		sss.move("torso","home")
		#move neck/base
		#get map

		return 'succeeded'

"""experimental state, should be replaces by generic state for approach pose"""
class ApproachScanPose(smach.State):

    def __init__(self):

        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['scan_pose']) #good angle value: 0.4

    def execute(self, userdata):
        scan_pose = userdata.scan_pose #[-1.3, -1.0, 3.14]
        sss.move("torso","home")
        sss.move("head","front")
        #sss.move("tray","down")
        #sss.move("base",scan_pose)

        return 'succeeded'

class Map360(smach.State):

	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['succeeded', 'failed'],
			input_keys=['angle_range']) #good angle value: 0.4
		self.client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)

	def execute(self, userdata):
		scan_pose = [0, 0, 0]
		sss.move("base",scan_pose)
		sss.move("torso","home")
		sss.move("head","front")
		sss.move("tray","down")
		#sss.move("base",scan_position)
		goal = TriggerMappingGoal()
		goal.start = True
		if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
			rospy.logerr('server not available')
			return 'failed'
		self.client.send_goal(goal)
		if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
			return 'failed'
		i = 0.2
		ctr = 0
		while i <= 6.2:
			scan_pose[2]=i
			sss.move("base",scan_pose)
			if operator.mod(ctr,2) == 0:
				sss.move("torso",[[-0.2,0.0,-0.2]])
			else:
				sss.move("torso","home")
			#sss.sleep(0.5)
			i = i+0.2
			ctr = ctr+1
		goal.start = False
		self.client.send_goal(goal)
		self.client.wait_for_result(rospy.Duration.from_sec(5.0))
		#sss.move("torso","home")
		#move neck/base
		#get map

		return 'succeeded'
