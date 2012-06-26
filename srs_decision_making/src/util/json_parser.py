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
# Author: Ze Ji, Renxi Qiu
#
# \date Date of creation: June 2012
#
# \brief
# parsing json commands
#
#################################################################

import roslib 
roslib.load_manifest('srs_decision_making')
#import json
import rospy
import simplejson as json

def decode_action(json_param):
    """
    return the action name
    """
    try:
        json_decoded = json.loads(json_param)
    
        act = json_decoded['action']
    except json.JSONDecodeError:
        print "Oops, error when decoding json_param", json_param
        return None

    return act 

def decode_move_parameters(json_param):
    """
    return a dict containing pose2d of the destination
    """
    try:
        json_decoded = json.loads(json_param)
    
        json_dest = json_decoded['destination']
        json_pose2d = json_dest['pose2d']

        #pose_dict = dict()
    
        #pose_dict['x'] = json_pose2d['x']
        #pose_dict['y'] = json_pose2d['y']
        #pose_dict['theta'] = json_pose2d['theta']
    except json.JSONDecodeError:
        print "Oops, error when decoding json_param", json_param
        return None

    return json_pose2d

def decode_detect_parameters(json_param):
    """
    return a dict containing object info (name etc)
    """
    try:
        json_decoded = json.loads(json_param)
    
        json_obj = json_decoded['object']
        #json_obj_type = json_obj['object_type']
    except json.JSONDecodeError:
        print "Oops, error when decoding json_param", json_param
        return None

    return json_obj

def decode_grasp_parameters(json_param):
    """
    return a dict containing object info (name etc)
    """
    try:
        json_decoded = json.loads(json_param)
    
        json_obj = json_decoded['object']
        #json_obj_type = json_obj['object_type']
    except json.JSONDecodeError:
        print "Oops, error when decoding json_param", json_param
        return None

    return json_obj

def decode_check_ws_parameters(json_param):
    """
    return a dict containing workspace info (name etc) -- check workspace
    """
    try:
        json_decoded = json.loads(json_param)
    
        json_obj = json_decoded['workspace']
        #json_obj_type = json_obj['object_type']
        json_dest = json_decoded['destination']
        json_pose2d = json_dest['pose2d']

    except json.JSONDecodeError:
        print "Oops, error when decoding json_param", json_param
        return None

    return json_obj, json_pose2d

def detect_feedback_to_json(userdata):
    jsonPose = ''
   
    pose = dict()
    pose['x'] = userdata.target_object_pose.pose.position.x
    pose['y'] = userdata.target_object_pose.pose.position.y
    pose['z'] = userdata.target_object_pose.pose.position.z
    pose['rotx'] = userdata.target_object_pose.pose.orientation.x
    pose['roty'] = userdata.target_object_pose.pose.orientation.y
    pose['rotz'] = userdata.target_object_pose.pose.orientation.z
    pose['rotw'] = userdata.target_object_pose.pose.orientation.w

    feedback = dict()
    feedback['action'] = 'detect'

    jsonObjectType = dict()
    jsonObjectType['object_type'] = userdata.target_object_name
    feedback['object'] = jsonObjectType
    feedback['pose'] = pose

    wrapAll = dict()
    wrapAll['feedback'] = feedback

    jsonPose = json.dumps(wrapAll)

    return jsonPose

class Task:
    def __init__(self, json_task):
        self.json_task = json_task
        self.task_json_string = json.dumps(self.json_task)

    def addItem(self, key, value):
        self.json_task[key] = value
        self.task_json_string = json.dumps(self.json_task)

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


class Task_Feedback:
    def __init__(self, task_id, task_initializer, task_initializer_type, task_json_string):
        self.task_id = task_id
        self.task_initializer = task_initializer
        self.task_initializer_type = task_initializer_type
        self.task_name = ''  #to be updated after decoding
        self.task_parameter = '' #to be updated after decoding
        self.task_schedule = '' #to be updated after decoding
        self.json_decoded = json.loads(task_json_string)
        #print self.json_decoded
        
        self.decode()
    
    def decode(self):
        
        self.task_name = self.json_decoded['task']
        
        if 'task_schedule' in self.json_decoded:
            self.task_schedule = self.json_decoded['task_schedule']
            
        
        command_list = ['search','get','fetch','deliver', 'move']
        
        if self.task_name.lower() in command_list:
            if    self.task_name.lower() == 'move':
                destination = self.json_decoded['destination']
                
                print destination
                
                if 'pose2d_string' in destination:
                    self.task_parameter = destination ['pose2d_string']
                elif 'predefined_pose' in destination:
                    self.task_parameter = destination ['predefined_pose']
                elif 'pose2d' in destination:
                    self.task_parameter = destination ['pose2d']
                else:
                    print 'WARNING not understandable destination found for move'
            else:
                try:
                    self.task_parameter = self.json_decoded['object']['object_type']
                except keyError:
                     print 'WARNING no target object given for search, get, fetch or deliver'
        else:
            print 'WARNING not understandable command found, available commands are'
            print command_list
        
        
        

"""
JASON request formation
            
=============== To Move to Coordinate in one single string format at time 1263798000000 ==================

{"tasks":[{"time_schedule":1263798000000,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== To Move to a predefined position (immediately, no time specified) ==================

{"tasks":[{"task":"move","destination":{"predefined_pose":"sofa_right"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== To Move to Coordinate by specifying the pose parameters (as numbers) (immediately, no time specified) ==================

{"tasks":[{"task":"move","destination":{"pose2d":{"theta":3.14,"y":1.0,"x":0.0}}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== To Fetch an object to a predefined position (soft_left) ==================

{"tasks":[{"time_schedule":1263798000000,"task":"fetch","deliver_destination":{"predefined_pose":"sofa_left"},"object":{"object_type":"Milkbox"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== To Fetch an object to a predefined position (soft_left), from a list of possible workspaces (Table0, Table1) ==================

{"tasks":[{"task":"fetch","workspaces":["Table0","Table1"],"deliver_destination":{"predefined_pose":"sofa_left"},"object":{"object_type":"Milkbox"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== Similar to Fetch, To Get a Book ==================

{"tasks":[{"task":"get","object":{"object_type":"Book"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== Similar to Fetch, To Get a Book from a list of workspaces (BookShelf0, Table0) ==================

{"tasks":[{"task":"get","workspaces":["BookShelf0","Table0"],"object":{"object_type":"Book"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}


=============== User specifies a list of tasks 1) To Get a MilkBox 2) move back to sofa_right with different time schedules ==================

{"tasks":[{"time_schedule":1263798000000,"task":"get","object":{"object_type":"Milkbox"}},{"time_schedule":1263798000000,"task":"move","destination":{"predefined_pose":"sofa_right"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}

"""          
            
            
            
"""
JASON feedback formation
            
            
current_action: the action excuting in the robot at the moment
    name: the name of the state machine  e.g. sm_srs_navigation, sm_srs_detection, sm_srs_grasp, sm_srs_put_on_tray, sm_enviroment_update
    state: the state of the operation e.g. started, completed
    step_id: the step in the action sequence

feedback: description message send back to UI
    lang: the language of the message  e.g. en, it, de
    message: the message itself

last_action: the information of the previous action
    name: the name of the state machine  e.g. sm_srs_navigation, sm_srs_detection, sm_srs_grasp, sm_srs_put_on_tray, sm_enviroment_update    
    outcome: the outcome of the operation e.g. succeeded, not_completed, failed, preempted

task: the information of the overall task operated by the robot
    task_id: a unique id of the task
    task_initializer: the device id of the ui which initialised the task
    task_initializer_type: the type of the device which initialised the task
    task_name: the name of the overall task  e.g. get, fetch, move
    task_parameter: the main parameter of the overall task  e.g. milk

Example:

[
  {
    "current_action": {
      "name": "sm_srs_navigation",
      "state": "started",
      "step_id": 3
    },
    "feedback": {
      "lang": "en",
      "message": "navigation started"
    },
    "last_action": {
      "name": "sm_srs_detection",
      "outcome": "succeeded",
      "step_id": 2
    },
    "task": {
      "task_id": "dm_10001_1",
      "task_initializer": "ui_loc_0001",
      "task_initializer_type": "ui_loc",
      "task_name": "fetch",
      "task_parameter": "milk"
    }
  }
]      
            
"""  
