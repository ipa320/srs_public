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
# \date Date of creation: January 2013
#
# \brief
# This module records topics according to triggering events
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

import roslib
PKG = "srs_user_tests"
roslib.load_manifest(PKG)
import rospy

import rosbag
from std_msgs.msg import Int32, String

import tf
from tf.msg import *
from tf.transformations import euler_from_quaternion

from simple_script_server import *
sss = simple_script_server()

from kinematics_msgs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from move_base_msgs.msg import *
from visualization_msgs.msg import *

import math
import rostopic
import time

import os
import sys, subprocess

import joint_states_aggregator
import tf_aggregator

import global_lock

class record_topic():

    def __init__(self, topic_name, bagfile, continuous=False):
          # this gets the topics type using the roslib
        the_type = rostopic.get_topic_type(topic_name, blocking=True)[0]

        the_type = the_type.split("/")
        the_type[0] += ".msg"
        
        # this imports the necessary modules and instantiates the necessary object
        mod = __import__(the_type[0], fromlist=[the_type[1]])
        cls = getattr( mod , the_type[1] )
        
        self.msg = None
        self.topic_name = topic_name 
        self.topic_type = cls
        self.continuous = continuous
        self.bagfile = bagfile

        # this creates the subscriber for the specific topic
        rospy.Subscriber(self.topic_name, self.topic_type, self.callback)
        global_lock.locked = False
        
    def lock(self):
        global_lock.locked = True
        
    def unlock(self):
        global_lock.locked = False     
        
    def callback(self, msg):
        self.msg = msg
        if self.continuous:
            self.record()
    
    def record(self):
        if(self.msg!=None):
            if not global_lock.locked:
                try:
                    if(global_lock.active_bag):
                        self.lock()
                        self.bagfile.write(self.topic_name, self.msg)
                        self.unlock()
                except KeyError, e:
                    rospy.loginfo(self.msg)
        
if __name__ == "__main__":

	rospy.init_node('record_topic')
	topics_list = ["/tf", "/collision_velocity_filter/velocity_limited_marker", "/joint_states"]
	topics_c = []
	
	for top in topics_list:
	    topic_r = record_topic(top)
	    
	    topics_c.append(topic_r)
	
	while not rospy.is_shutdown():
	
	    for tops_c in topics_c:
	        rospy.loginfo(tops_c.msg)
	    rospy.sleep(2)
	
