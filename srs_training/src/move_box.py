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
# This is part of the SRS training package
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
import roslib; roslib.load_manifest('srs_training')
import rospy
from geometry_msgs.msg import Twist
import tf
from math import *


class move_box():

    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.reference_frame = "/base_link"
        self.target_frame = "/map"
        rospy.Subscriber("/base_controller/command", Twist, self.callback)

        self.x = 0.
        self.y = 0.
        self.z = 0.2
        self.r = 0.
        self.p = 0.
        self.t = 0.
        self.twist = Twist()

        rospy.sleep(2)
        self.tf_broadcaster.sendTransform((self.x, self.y, self.z),
            tf.transformations.quaternion_from_euler(self.r, self.p, self.t),
            rospy.Time.now(),
            self.reference_frame,
            self.target_frame)

        self.current_time = rospy.get_time()
        self.last_time = rospy.get_time()

    def callback(self,msg):
        self.twist = msg

if __name__ == '__main__':
    rospy.init_node('move_box')
    mb = move_box()
    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        mb.current_time = rospy.get_time()

        try:
            (trans,rot) = mb.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        x = (mb.current_time - mb.last_time) * (mb.twist.linear.x * cos(euler[2]) - mb.twist.linear.y * sin(euler[2])) + trans[0]
        y = (mb.current_time - mb.last_time) * (mb.twist.linear.x * sin(euler[2]) + mb.twist.linear.y * cos(euler[2])) + trans[1]
        t = mb.twist.angular.z*(mb.current_time - mb.last_time) + euler[2]

        mb.tf_broadcaster.sendTransform((x , y, 0),
            tf.transformations.quaternion_from_euler(0, 0, t),
            rospy.Time.now(),
            mb.reference_frame,
            mb.target_frame)
        mb.last_time = mb.current_time
        rate.sleep()
