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

import itertools

import os
import sys, subprocess

import record_topic

class topics_bag():

    def __init__(self):

        # this defines the variables according to the ones specified at the yaml
        # file. The triggers, the wanted tfs, the wanted topics and where they are going
        # to be written, more specifically at the file named as self.bag.
        self.trigger_record_translation = rospy.get_param('~trigger_record_translation')
        self.trigger_record_rotation = rospy.get_param('~trigger_record_rotation')
        self.record_timestep = rospy.get_param('~record_timestep')
        self.record_timestep_tf = rospy.get_param('~record_timestep_tf')
        self.wanted_tfs = rospy.get_param('~wanted_tfs')
        self.wanted_topics = rospy.get_param("~wanted_topics")
        self.bag_name = rospy.get_param("~bag_name")
    
        # this creates the bagfile
        localtime = time.localtime(time.time())
        filename = self.bag_name + "_" + str(localtime[0]) + "-" + str(localtime[1]) + "-" + str(localtime[2]) + "_" + str(localtime[3]) + "-" + str(localtime[4]) + "-" + str(localtime[5]) + ".bag"
        filelocation = str(roslib.packages.get_pkg_dir(PKG) + "/data/" )
        rospy.loginfo("Logging to " + filelocation + filename)
        self.bag = rosbag.Bag(filelocation + filename, 'w')
        
        # necessary tf elements 
        self.tfL = tf.TransformListener()
        self.tfposed = TransformStamped()
        self.tfMsg = tfMessage()
        
        # waits for a tf transform before starting. This is important to check if
        # the system is fully functional.
        
        now = rospy.Time.now()
        self.tfL.waitForTransform(self.wanted_tfs[0]["reference_frame"], self.wanted_tfs[0]["target_frame"], now, rospy.Duration(4.0))
        
        # dictionaries for storing current translation and rotation for the specific
        # frames
        self.current_translation = {}
        self.current_rotation = {}
        
        for frame in self.wanted_tfs:
            self.current_translation[frame["target_frame"]] = [0,0,0]
            self.current_rotation[frame["target_frame"]] = [0,0,0,0]

        
    def tf_trigger(self, reference_frame, target_frame, tfs):
        #  this function is responsible for setting up the triggers for recording
        # on the bagfile.
        
        # sequence for calculating distance and yaw rotation for defining if a 
        # recording trigger is set according to the trigger value on the yaml file
        now = rospy.Time.now()
        self.tfL.waitForTransform(reference_frame, target_frame, now, rospy.Duration(3.0))
        trans, rot = self.tfL.lookupTransform(reference_frame, target_frame, now)
        
        x = trans[0] - self.current_translation[target_frame][0]
        y = trans[1] - self.current_translation[target_frame][1]
        
        distance_trans = math.sqrt(x*x + y*y)
        distance_rot = abs(euler_from_quaternion(rot)[2] - euler_from_quaternion(self.current_rotation[target_frame])[2])
        
        if("trigger_record_translation" in tfs and distance_trans >= tfs["trigger_record_translation"]):
            rospy.loginfo("triggered for translation, trans = " + str(distance_trans))
            self.current_translation[target_frame] = trans
            self.current_rotation[target_frame] = rot
            return "triggered"

        if("trigger_record_rotation" in tfs and distance_rot >= tfs["trigger_record_rotation"]):
            rospy.loginfo("triggered for rotation, rot = " + str(distance_rot))
            self.current_translation[target_frame] = trans
            self.current_rotation[target_frame] = rot
            return "triggered"
        
        return "not_triggered"
        

    def bag_processor(self, tfs=None):
        
        trigger_position = self.tf_trigger(tfs["reference_frame"], tfs["target_frame"], tfs)
        
        return trigger_position

if __name__ == "__main__":

	rospy.init_node('topics_bag')

	bagR = topics_bag()

	time_step = rospy.Duration.from_sec(bagR.record_timestep)
	time_step_tf = rospy.Duration.from_sec(bagR.record_timestep_tf)
	start_time_tf = rospy.Time.now()
	start_time = rospy.Time.now()

	with bagR.bag as bagfile:

		rate = rospy.Rate(10) #Hz
		topics_c = []
		for tfs in bagR.wanted_topics:
		    topic_r = record_topic.record_topic(tfs)
		    topics_c.append(topic_r)
		    
		while not rospy.is_shutdown():
			# listen to tf changes
			for tfs in bagR.wanted_tfs:
				triggers = bagR.bag_processor(tfs)
				if(triggers == "triggered"):
					rospy.loginfo("triggered")
					start_time = rospy.Time.now()
					for tops_c, tfm in itertools.izip(topics_c, bagR.wanted_topics):
					    if(tops_c.msg!=None):
    						bagfile.write(tfm, tops_c.msg)
				else:
					rospy.logdebug("not triggered")
			
			# listen to ellapsed time
			time_msg = "time passed:" + (str)((rospy.Time.now() - start_time).to_sec())
			rospy.logdebug(time_msg)
			if(rospy.Time.now() - start_time > time_step):
				rospy.loginfo("triggered by time")
				start_time = rospy.Time.now()
				for tops_c, tfm in itertools.izip(topics_c, bagR.wanted_topics):
				        if(tops_c.msg!=None):
    						bagfile.write(tfm, tops_c.msg)
			
			# sleep until next check
			#if(rospy.Time.now() - start_time_tf > time_step_tf):
		#		rospy.logdebug("Forcing tf record")
	#			start_time_tf = rospy.Time.now()
#				bagR.process_topics("/tf")
			rate.sleep()
			
	# closing bag file
	rospy.loginfo("Closing bag file")
	bagR.bag.close()
