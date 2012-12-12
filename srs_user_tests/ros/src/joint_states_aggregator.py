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
# This module aggregates joint states
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

import threading

class joint_state_aggregator():

    def __init__(self):
        # lock object is necessary to prevent interruption on the joint_states message
        # update process
        self.lock = threading.Lock()
        # the following lists store the components of the joint_states message
        self.effort = []
        self.position = []
        self.velocity = []
        self.names = []
        # joint_states message
        self.jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState, 10)
        # this checks for the dimensions of the message to inform if there is a 
        # malformed joint_states message for the developer to be able to correct
        # the related controller code
        if(len(self.jointsMsg.name) == len(self.jointsMsg.position) == len(self.jointsMsg.velocity) == len(self.jointsMsg.effort)):
            rospy.logdebug("Dimensions of joint_state message are ok!")
        else:
            excep = "Joint_states dimensions are not correct:" + "Names_dim " + (str)(len(self.jointsMsg.name)) + "," + "Pos_dim " + (str)(len(self.jointsMsg.position)) + "," + "Vel_dim " + (str)(len(self.jointsMsg.velocity)) + "," + "Eff_dim " + (str)(len(self.jointsMsg.effort))
            raise Exception(excep)
        # iterates simultaneously through all the message lists to update the correspondent
        # values according to the new joint_states message
        for a,b,c, d in itertools.izip(self.jointsMsg.name, self.jointsMsg.position,self.jointsMsg.velocity, self.jointsMsg.effort):
            self.names.append(a)
            self.velocity.append(b)
            self.position.append(c)
            self.effort.append(d)
        # establishes a subscriber and a callback to the joint_states
        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.process_joints)
        # creates a thread for managing the joint_states messages listener
        #self.thread = threading.Thread(target=self.joint_states_listener)
        #self.thread.start()
        
    def joint_states_listener(self):
        rospy.Subscriber('/joint_states', JointState, self.process_joints)
        rospy.spin()
        
    def process_joints(self, msg):
        self.lock.acquire()
        jMsg = msg#rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState, 10)
        self.temp_name = jMsg.name
        self.temp_vel = jMsg.velocity
        self.temp_pos = jMsg.position
        self.temp_ef = jMsg.effort
        # dimensions check
        if(len(jMsg.name) == len(jMsg.position) == len(jMsg.velocity) == len(jMsg.effort)):
            pass
        else:
            excep = "Joint_states dimensions are not correct:" + "Names_dim " + (str)(len(jMsg.name)) + "," + "Pos_dim " + (str)(len(jMsg.position)) + "," + "Vel_dim " + (str)(len(jMsg.velocity)) + "," + "Eff_dim " + (str)(len(jMsg.effort)) + "joint_names = " + str(jMsg.name)
            raise Exception(excep)
        # iterates through the message items to update the corresponding lists
        for a,b,c, d in itertools.izip(jMsg.name, jMsg.position,jMsg.velocity, jMsg.effort):
        
            if(a) not in self.names:
                self.names.append(a)
                self.position.append(b)
                self.velocity.append(c)
                self.effort.append(d)
            
            # this is important for updating the current items that are already
            # on the lists
            self.position[self.names.index(a)] = self.temp_pos[self.temp_name.index(a)]
            self.velocity[self.names.index(a)] = self.temp_vel[self.temp_name.index(a)]
            self.effort[self.names.index(a)] = self.temp_ef[self.temp_name.index(a)]
        # this creates the aggregated joint_states message and makes it available
        # when requested
        self.jointsMsg.name = self.names
        self.jointsMsg.position = self.position
        self.jointsMsg.velocity = self.velocity
        self.jointsMsg.effort = self.effort
        
        self.lock.release()
        #return self.jointsMsg
        
if __name__=="__main__":

    rospy.init_node('joint_test')
    ja = joint_state_aggregator()
    rate = rospy.Rate(10) #Hz
    
    while not rospy.is_shutdown():
        
        #ja.process_joints()
        
        rospy.loginfo("Current message:")
        rospy.loginfo(ja.jointsMsg)
        rospy.loginfo("Collected names:")
        rospy.loginfo(ja.names)
        rospy.loginfo(ja.position)
        rospy.loginfo(ja.velocity)
        
        rate.sleep()
