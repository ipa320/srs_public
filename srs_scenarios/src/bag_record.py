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
# \date Date of creation: October 2012
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
from sensor_msgs.msg import *
from move_base_msgs.msg import *

import math
import rostopic

import os
import sys, subprocess

class bag_record():

    def __init__(self):

        self.trigger_record_translation = rospy.get_param('trigger_record_translation')
        self.trigger_record_angle = rospy.get_param('trigger_record_angle')
        self.record_timestep = rospy.get_param('record_timestep')
        self.wanted_tfs = rospy.get_param('wanted_tfs')
        self.wanted_topics = rospy.get_param("wanted_topics")
        self.bag_name = rospy.get_param("bag_name")
    
        self.bag = rosbag.Bag(self.bag_name, 'w')
        
        self.tfL = tf.TransformListener()
        self.tfposed = TransformStamped()
        self.tfMsg = tfMessage()
        
        self.tfL.waitForTransform(self.wanted_tfs[0]["reference_frame"], self.wanted_tfs[0]["target_frame"], rospy.Time(), rospy.Duration(20.0))
        
        self.current_translation = {}
        self.current_rotation = {}
        
        for frame in self.wanted_tfs:
            self.current_translation[frame["target_frame"]] = [0,0,0]
            self.current_rotation[frame["target_frame"]] = [0,0,0,0]
        
    def tf_trigger(self, reference_frame, target_frame, tfs):
        
        trans, rot = self.tfL.lookupTransform(reference_frame, target_frame, rospy.Time(0))
        
        x = trans[0] - self.current_translation[target_frame][0]
        y = trans[1] - self.current_translation[target_frame][1]
        
        distance = x*x + y*y
        distance = math.sqrt(distance)
        
        
        self.tfposed.header.frame_id = target_frame
        self.tfposed.header.stamp = rospy.Time.now()
        self.tfposed.child_frame_id = reference_frame          
        self.tfposed.transform.translation.x = trans[0]
        self.tfposed.transform.translation.y = trans[1]
        self.tfposed.transform.translation.z = trans[2]
        self.tfposed.transform.rotation.x = rot[0]
        self.tfposed.transform.rotation.y = rot[1]
        self.tfposed.transform.rotation.z = rot[2]
        self.tfposed.transform.rotation.w = rot[3]
        
        self.tfMsg = tfMessage([self.tfposed])
        
        if("trigger_record_translation" in tfs and distance >= tfs["trigger_record_translation"]):
            self.current_translation[target_frame] = trans
            self.current_rotation[target_frame] = rot
            return "triggered"
        else:
            return "not_triggered"
    
    def process_topics(self, tfs):
        
        the_type = rostopic.get_topic_type(tfs, blocking=True)[0]

        the_type = the_type.split("/")
        the_type[0] += ".msg"
        
        
        mod = __import__(the_type[0], fromlist=[the_type[1]])
        cls = getattr( mod , the_type[1] )
        
        msg =  rospy.wait_for_message(tfs, cls)
        
        return msg
    
    def bag_processor(self, tfs=None):
        
        print tfs
        trigger_position = self.tf_trigger(tfs["reference_frame"], tfs["target_frame"], tfs)
        
        return trigger_position

if __name__ == "__main__":

    rospy.init_node('bag_recording')
    
    bagR = bag_record()
    
    time_step = bagR.record_timestep
    start_time = rospy.rostime.get_time()
    
    with bagR.bag as bagfile:

	while not rospy.is_shutdown():
        
		for tfs in bagR.wanted_tfs:
		    triggers = bagR.bag_processor(tfs)
		    if(triggers == "triggered"):
		        rospy.loginfo("triggered")
		        bagfile.write("/tf", bagR.tfMsg)
		        for tfs in bagR.wanted_topics:
		            print "triggered topic"
		            msg = bagR.process_topics(tfs)
		            bagfile.write(tfs, msg)
		        start_time = rospy.rostime.get_time()
		    else:
		        rospy.loginfo("not triggered")
		time_msg = "time passed:" + (str)(rospy.rostime.get_time() - start_time)
		rospy.loginfo(time_msg)
		if(rospy.rostime.get_time() - start_time > time_step):
		        rospy.loginfo("triggered by time")
		        for tfs in bagR.wanted_tfs:
		            fake_trigger = bagR.bag_processor(tfs)
		            bagfile.write("/tf", bagR.tfMsg)
		        for tfs in bagR.wanted_topics:
		            rospy.loginfo("triggered topic with time")
		            msg = bagR.process_topics(tfs)
		            bagfile.write(tfs, msg)
		        start_time = rospy.rostime.get_time()    
