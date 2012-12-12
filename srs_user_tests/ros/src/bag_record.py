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

class bag_record():

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
        self.tfL.waitForTransform(self.wanted_tfs[0]["reference_frame"], self.wanted_tfs[0]["target_frame"], rospy.Time(), rospy.Duration(3.0))
        
        # dictionaries for storing current translation and rotation for the specific
        # frames
        self.current_translation = {}
        self.current_rotation = {}
        
        # aggregator objects. Those are important for aggregatinf malformed/not 
        # complete messages for future recording.
        self.ja = joint_states_aggregator.joint_state_aggregator()
        self.tfa = tf_aggregator.tf_aggregator()
        
        # subscribers
        rospy.Subscriber('/collision_velocity_filter/velocity_limited_marker', Marker, self.velocity_limited_marker_callback)
        self.base_velocity_limited_marker = Marker()
        self.new_base_velocity_limited_marker = False
        
        for frame in self.wanted_tfs:
            self.current_translation[frame["target_frame"]] = [0,0,0]
            self.current_rotation[frame["target_frame"]] = [0,0,0,0]

    def velocity_limited_marker_callback(self,msg):
        self.base_velocity_limited_marker = msg
        self.new_base_velocity_limited_marker = True
#        if msg.id == 0 or msg.id == 1:
#            self.bag.write("/logger/base_collision_x", log_msg)
#        elif msg.id == 2 or msg.id == 3:
#            self.bag.write("/logger/base_collision_y", log_msg)
#        elif msg.id == 4 or msg.id == 5:
#            self.bag.write("/logger/base_collision_yaw", log_msg)
#        else:
#            rospy.logerror("invalid marker id, check ids in cob_collision_velocity_filter")
        
    def tf_trigger(self, reference_frame, target_frame, tfs):
        #  this function is responsible for setting up the triggers for recording
        # on the bagfile.
        
        # sequence for calculating distance and yaw rotation for defining if a 
        # recording trigger is set according to the trigger value on the yaml file
        trans, rot = self.tfL.lookupTransform(reference_frame, target_frame, rospy.Time(0))
        
        x = trans[0] - self.current_translation[target_frame][0]
        y = trans[1] - self.current_translation[target_frame][1]
        
        distance_trans = math.sqrt(x*x + y*y)
        distance_rot = abs(euler_from_quaternion(rot)[2] - euler_from_quaternion(self.current_rotation[target_frame])[2])
        
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
        
    def tf_write(self, transf):
        # writes tf messages on the bagfile
        rospy.logdebug("Tf_writer")
        self.tfa.lock()
        # loops through the transforms dictionaries and get the current transforms
        # for the specific frames and further records the messages on the bagfile
        for t in transf:
            for j in transf[t]:
                #rospy.loginfo(t)
                #rospy.loginfo(j)
                trans, rot = self.tfL.lookupTransform(t, j, rospy.Time(0))
                
                self.tfposed.header.frame_id = t
                self.tfposed.header.stamp = rospy.Time.now()
                self.tfposed.child_frame_id = j
                self.tfposed.transform.translation.x = trans[0]
                self.tfposed.transform.translation.y = trans[1]
                self.tfposed.transform.translation.z = trans[2]
                self.tfposed.transform.rotation.x = rot[0]
                self.tfposed.transform.rotation.y = rot[1]
                self.tfposed.transform.rotation.z = rot[2]
                self.tfposed.transform.rotation.w = rot[3]
                
                tfMsg = tfMessage([self.tfposed])

                bagfile.write("/tf", tfMsg)
        self.tfa.unlock()
    
    def process_topics(self, tfs):
        # process the topics
        
        # this gets the topics type using the roslib
        the_type = rostopic.get_topic_type(tfs, blocking=True)[0]

        the_type = the_type.split("/")
        the_type[0] += ".msg"
        
        # this imports the necessary modules and instantiates the necessary object
        mod = __import__(the_type[0], fromlist=[the_type[1]])
        cls = getattr( mod , the_type[1] )
        
        timeout = 3.0
        try:
            if (tfs == "/joint_states"):
                msg = self.ja.jointsMsg
                #rospy.loginfo("joint Message")
                #rospy.loginfo(msg)
            
            elif(tfs=="/tf"):
                #self.tfa.process_tfs()
                transf = self.tfa.transforms
                self.tf_write(transf)
                    
                msg = Empty()
                
            else:
                msg =  rospy.wait_for_message(tfs, cls, timeout)
        except rospy.ROSException:
            # an empty message is created due to an exception for preventing breaks
            # on the recording process
        	rospy.logwarn("skipping topic: " + str(tfs))
        	msg = Empty()
        	
        return msg

    def bag_processor(self, tfs=None):
        
        trigger_position = self.tf_trigger(tfs["reference_frame"], tfs["target_frame"], tfs)
        
        return trigger_position

if __name__ == "__main__":

	rospy.init_node('bag_recording')

	bagR = bag_record()

	time_step = rospy.Duration.from_sec(bagR.record_timestep)
	time_step_tf = rospy.Duration.from_sec(bagR.record_timestep_tf)
	start_time_tf = rospy.Time.now()
	start_time = rospy.Time.now()

	with bagR.bag as bagfile:

		rate = rospy.Rate(10) #Hz
		while not rospy.is_shutdown():
			# listen to tf changes
			for tfs in bagR.wanted_tfs:
				triggers = bagR.bag_processor(tfs)
				if(triggers == "triggered"):
					rospy.loginfo("triggered")
					start_time = rospy.Time.now()
					# log tf
					#bagfile.write("/tf", bagR.tfMsg)
					# log all topics
					for tfs in bagR.wanted_topics:
						#print "triggered topic"
						msg = bagR.process_topics(tfs)
						if(tfs != "/tf"):
							bagfile.write(tfs, msg)
				else:
					rospy.logdebug("not triggered")
			
			# listen to ellapsed time
			time_msg = "time passed:" + (str)((rospy.Time.now() - start_time).to_sec())
			rospy.logdebug(time_msg)
			if(rospy.Time.now() - start_time > time_step):
				rospy.loginfo("triggered by time")
				start_time = rospy.Time.now()
				# log tf
				for tfs in bagR.wanted_tfs:
					fake_trigger = bagR.bag_processor(tfs)
					#bagfile.write("/tf", bagR.tfMsg)
				# log all topics
				for tfs in bagR.wanted_topics:
					#rospy.loginfo("triggered topic with time")
					msg = bagR.process_topics(tfs)
					if(tfs!="/tf"):
						bagfile.write(tfs, msg)

			# log base collision velocity limited marker
			if bagR.new_base_velocity_limited_marker:
				bagfile.write("/collision_velocity_filter/velocity_limited_marker", bagR.base_velocity_limited_marker)
				bagR.new_base_velocity_limited_marker = False
			
			# sleep until next check
			if(rospy.Time.now() - start_time_tf > time_step_tf):
				rospy.logdebug("Forcing tf record")
				start_time_tf = rospy.Time.now()
				bagR.process_topics("/tf")
				#msg =  rospy.wait_for_message("/tf", tfMessage)
				#bagfile.write("/tf", msg)
			rate.sleep()
	# closing bag file
	rospy.loginfo("Closing bag file")
	bagR.bag.close()
