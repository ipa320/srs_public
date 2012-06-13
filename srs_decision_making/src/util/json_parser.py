#!/usr/bin/env python
#################################################################
##\file
#
# \note
# Copyright (c) 2011, 2012 \n
# Cardiff University \n
#
#################################################################
#
# \note
# Project name: Multi-Role Shadow Robotic System for Independent Living
# \note
# ROS stack name: srs
# \note
# ROS package name: srs_decision_making
#
# \author
# Author: Ze Ji
#
# \date Date of creation: June 2012
#
# \brief
# parsing json commands
#
#################################################################

import roslib 
roslib.load_manifest('srs_decision_making')
import json
import rospy

class Task:
    def __init__(self, json_raw_string):
        ## may not be needed , as this will be handled by the knowledge service
        print json_raw_string
        this.json_raw_string = json_raw_string

class Tasks:

    def __init__(self, json_raw_string):
        self.json_raw_string = json_raw_string
        self.json_decoded = json.loads(self.json_raw_string)
        self.tasks_json = []
        self.tasks = []
        self.device_id = ''   # invalid default empty
        self.device_type = '' # empty
        self.decode()

    def decode(self):
        
        self.device_id = self.json_decoded['initializer']['device_id']
        self.device_type = self.json_decoded['initializer']['device_type']
        self.tasks_json = str(self.json_decoded['tasks'])
        self.tasks = self.json_decoded['tasks']
