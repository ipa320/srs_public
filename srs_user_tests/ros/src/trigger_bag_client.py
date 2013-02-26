#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2013 \n
# Fraunhofer Institute for Manufacturing Engineering
# and Automation (IPA) \n\n
#
#################################################################
#
# \note
# Project name: Care-O-bot Research
# \note
# ROS package name: 
#
# \author
# Author: Thiago de Freitas Oliveira Araujo, 
# email:thiago.de.freitas.oliveira.araujo@ipa.fhg.de
# \author
# Supervised by: Florian Weisshardt, email:florian.weisshardt@ipa.fhg.de
#
# \date Date of creation: February 2013
#
# \brief
# This module records bagfile according to triggering events
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
# - Neither the name of the Fraunhofer Institute for Manufacturing
# Engineering and Automation (IPA) nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission. \n
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
# If not, see < http://www.gnu.org/licenses/>.
#
#################################################################
import roslib; roslib.load_manifest('srs_user_tests')

import sys

import rospy
import rosservice
from std_msgs.msg import String

def bag_service_client_start():
    rospy.wait_for_service('/logger/start')
    try:
        rosservice.call_service('/logger/start', None)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def bag_service_client_stop():
    rospy.wait_for_service('/logger/stop')
    try:
        rosservice.call_service('/logger/stop', None)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
		
if __name__ == "__main__":
	while not rospy.is_shutdown():
		var = raw_input("0: stop, 1: start\n")
		if var == str(0):
			bag_service_client_stop()
		elif var == str(1):
			bag_service_client_start()
		else:
			print "wrong command"
