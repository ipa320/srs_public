#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# Cardiff University \n\n
#
#################################################################
#
# \note
# Project name: Multi-Role Shadow Robotic System for Independent Living
# \note
# ROS stack name: srs
# \note
# ROS package name: srs_decision_making_interface
#
# \author
# Author: Renxi Qiu, email: renxi.qiu@gmail.com
#
# \date Date of creation: Jan 2012
#
# \brief
# Interface for SRS decision making
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
#
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
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
import roslib; roslib.load_manifest('srs_decision_making_interface')

from srs_decision_making_interface.srv import srs_dm_ui_interface
import srs_decision_making_interface.msg as xmsg
from std_msgs.msg import *
import rospy
import actionlib

#client = ''

def srs_decision_making_ui_interface_handle(req):
    global client
    _goal=xmsg.srs_actionGoal()
    _goal.action=req.action
    _goal.parameter=req.parameter
    _goal.priority=int(req.priority)
    client.send_goal(_goal)    
    return client.gh.comm_state_machine.action_goal.goal_id.id

def srs_decision_making_ui_interface():

    s = rospy.Service('srs_decision_making_ui_interface', srs_dm_ui_interface, srs_decision_making_ui_interface_handle)
    print "Ready to translate request from UI to ROS network."
    rospy.spin()



if __name__ == "__main__":
    rospy.init_node('srs_decision_making_ui_interface')
    global client
    client = actionlib.SimpleActionClient('srs_decision_making_actions', xmsg.srs_actionAction)
    client.wait_for_server()
    srs_decision_making_ui_interface()