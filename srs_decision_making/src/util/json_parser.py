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
    def __init__(self, json_task):
        ## may not be needed , as this will be handled by the knowledge service
        self.json_task = json_task
        #print json_raw_string
        #self.json_raw_string = json_raw_string
        self.task_json_string = json.dumps(json_task)

    def addItem(self, key, value):
        self.json_task[key] = value
        self.task_json_string = json.dumps(json_task)

class Tasks:

    def __init__(self, json_raw_string):
        self.json_raw_string = json_raw_string
        self.json_decoded = json.loads(self.json_raw_string)
        self.tasks_json = []
        self.tasks_dec = []
        
        self.tasks_list = []
        self.device_id = ''   # invalid default empty
        self.device_type = '' # empty
        self.decode()

    def decode(self):
        
        self.device_id = self.json_decoded['initializer']['device_id']
        self.device_type = self.json_decoded['initializer']['device_type']
        #self.tasks_json = str(self.json_decoded['tasks'])
        self.tasks_dec = self.json_decoded['tasks']

        for t in self.tasks_dec:
            tempTask = Task(t)
            self.tasks_list.append(tempTask)
            #self.tasks_json.append(json.dumps(t))
            #self.tasks_json.append(str(t))
