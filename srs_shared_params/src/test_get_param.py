#!/usr/bin/env python 
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
# ROS package name: srs_shared_params
#
# \author
# Author: Ze Ji, email: JiZ1@cf.ac.uk
#
# \date Date of creation: April 2012
#
# \brief
#  (a place that stores some parameters that can be used and shared by srs components.)
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

import roslib;

roslib.load_manifest('srs_shared_params')
import sys
import rospy

def get_params():
    a = rospy.get_param("/dictionary/a")
    print a
    f = rospy.get_param("/float")
    print f
    d1 = rospy.get_param('/dictionary/a')    
    print d1
    #default_param = rospy.get_param('default_param', 'default_value')

    # fetch a group (dictionary) of parameters
    d2 = rospy.get_param('/dictionary')
    a, c = d2['a'], d2['c']
    print a, c

if __name__ == "__main__":
    get_params()
