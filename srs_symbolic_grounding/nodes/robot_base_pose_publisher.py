#!/usr/bin/env python 
#################################################################
##\file
#
# \note
# Copyright (c) 2012 \n
# University of Bedfordshire \n\n
#
#################################################################
#
# \note
# Project name: care-o-bot
# \note
# ROS stack name: srs_public
# \note
# ROS package name: srs_symbolic_grounding
#
# \author
# Author: Beisheng Liu, email:beisheng.liu@beds.ac.uk
# \author
# Supervised by: Dayou Li, email:dayou.li@beds.ac.uk
#
# \date Date of creation: Mar 2012
#
# \brief
# publish robot base pose
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
# - Neither the name of the University of Bedfordshire nor the names of its
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
# If not, see <http://www.gnu.org/licenses/>.
#
################################################################# 
import roslib
roslib.load_manifest('srs_symbolic_grounding')
from geometry_msgs.msg import *
import rospy
import tf
import csv




def publisher():
	rb_pose = Pose2D()
	pub = rospy.Publisher('robot_base_pose', Pose2D)
	rospy.init_node('publisher')
	listener = tf.TransformListener()
	listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))
	spamWriter = csv.writer(open('trajectory.csv', 'wb'), delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	while not rospy.is_shutdown():
		(trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
		rb_pose.x = trans[0]
		rb_pose.y = trans[1]
		rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
		rb_pose.theta = rb_pose_rpy[2]
		rospy.loginfo(rb_pose)
		pub.publish(rb_pose)
		spamWriter.writerow([rb_pose.x, rb_pose.y, rb_pose.theta])
		rospy.sleep(0.5)
if __name__ == '__main__':
	try: 
		publisher()
	except rospy.ROSInterruptException: pass

