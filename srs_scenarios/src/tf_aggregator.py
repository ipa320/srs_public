#!/usr/bin/env python

#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
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
# \date Date of creation: December 2012
#
# \brief
# This module aggregates tf messages
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
roslib.load_manifest('srs_scenarios')
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

import math
import rostopic

import os
import sys, subprocess

import itertools

class tf_aggregator():

    def __init__(self):
        
        self.transforms = {}
        
    def process_tfs(self):
        
        msg =  rospy.wait_for_message("/tf", tfMessage)
        # iterates through the message items to update the transforms dictionary
        # with the specified reference and child frame
        for trans in msg.transforms:
            frame_id = trans.header.frame_id
            child_frame_id = trans.child_frame_id
            
            if(frame_id not in self.transforms): 
                self.transforms[frame_id] = {}
            if(child_frame_id not in self.transforms[frame_id]):
                self.transforms[frame_id][child_frame_id] = True
        
if __name__=="__main__":

    rospy.init_node('tf_test')
    tfa = tf_aggregator()
    
    while not rospy.is_shutdown():
        
        tfa.process_tfs()
        
        rospy.sleep(2)
