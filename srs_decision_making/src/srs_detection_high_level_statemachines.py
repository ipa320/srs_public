#!/usr/bin/python
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
# ROS package name: srs_decision_making
#
# \author
# Author: Renxi Qiu, email: renxi.qiu@gmail.com
#
# \date Date of creation: April 2012
#
# \brief
# Task coordination and interfacing for SRS decision making
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
# ROS imports
import roslib; roslib.load_manifest('srs_decision_making')
import os

#import states within srs_decision_making
from srs_generic_states import *

#import states from srs_states package
from mapping_states import *
from detection_states  import *
from assisted_detection_states import *

#import service form semantic KB
from srs_knowledge.srv import *

#import other high level state machines
from srs_common_high_level_statemachines import *

#from generic_grasp_state import *

# This is come from srs_object_verification, the name should be updated to avoid confusion
#from generic_states import *

"""
This file contains high level state machines for detection in decision making.
The actual implementation of the states can be found from 
    srs_states packages

Depend on if there is any user client connected and the type of the user client, 
state machines below may switch between semi-autonomous or fully-autonomous mode mode.

If semi_autonomous_mode=True Generic states of user interventions will be loaded for error handling
Otherwise, generic states based on semantic kb for error handling will be loaded 

The default mode is semi_autonomous_mode=False assuming no UI connected to the robot

    sm_simple_detection()
    #simple detection with no user intervention and no environment confirmation

    sm_simple_detection_env()
    #simple detection with no user intervention but with environment confirmation
    
    sm_assisted_detection()
    #assisted detection with user intervention and no environment confirmation 
    
    sm_assisted_detection_env()
    #assisted detection with user intervention and environment confirmation    
"""

################################################################################
#Simple detection state machine
#
#robot would detect the object autonomously
#no environment confirmation is needed for this detection
#
#if the name/id is empty, then the robot will search all the objects in the field of view
#if the environment confirmation/verification is not needed, the target workspace name is not used
#in the simple detection, only the nearest object to the camera is detected
#
#the id of the object is not used by the detection state yet, this should be changed
################################################################################

class sm_simple_detection(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name', 'target_object_id', 'target_workspace_name'],
                                    output_keys=['target_object','target_object_pose'])

        self.max_retries = 5  # default value for max retries
        try:
            self.max_retries = rospy.get_param("srs/common/detection_max_retries")
        except Exception, e:
			rospy.INFO('can not read the parameter of max retries, use the default value')	

        with self:   
            smach.StateMachine.add('DETECT_OBJECT-simple', detect_object('',self.max_retries),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT-simple', 'no_more_retries':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose' })

################################################################################
#Assisted detection state machine
#
#Robot would detect the object autonomously first, and then wait for the use for confirmation and adjustment via bounding box
#No environment confirmation is needed for this detection
#
#If the target object name/id is empty, then the robot will search all the objects in the field of view
#if the environment confirmation/verification is not needed, the target workspace name is not used
#in the assisted detection, all detected objects will be processed, user will choice the best object from the list
#
#the id of the object is not used by the detection state yet, this should be changed
################################################################################

#detection assisted by remote operator or KB, they specify region of interest for detection
class sm_assisted_detection(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name', 'target_object_id', 'target_workspace_name'],
                                    output_keys=['target_object','target_object_pose'])

        self.userdata.target_object_list=''
        self.userdata.new_scan_pose = ''

        with self:
            smach.StateMachine.add('DETECT_OBJECT-assisted', detect_object_assited(),
                    transitions={'succeeded':'USER_INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object_list':'target_object_list' , 'object_id':'target_object_id'})
            
            smach.StateMachine.add('USER_INTERVENTION', user_intervention_on_detection(),
					transitions={'succeeded':'succeeded', 'bb_move':'BB_MOVE', 'give_up':'not_completed', 'failed':'failed', 'preempted':'preempted','retry':'DETECT_OBJECT-assisted'},
					remapping={'target_object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose', 'bb_pose':'new_scan_pose', 'target_object_list':'target_object_list'})
            
            smach.StateMachine.add('BB_MOVE', approach_pose_without_retry(),
					transitions={'succeeded':'DETECT_OBJECT-assisted', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
					remapping={'base_pose':'new_scan_pose'})	


################################################################################
#Confirm the workspace is exist and it is not empty
#
###############################################################################
class workspace_confirmation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_workspace_name'])
        self.userdata.verified_workspace_pose =""
        
        with self:
            smach.StateMachine.add('VERIFY_EXISTENCE_OF_THE_WORKSPACE', Verify_Object_by_Name(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_workspace_name',  'verfified_target_object_pose':'verified_workspace_pose'})
            """        
            smach.StateMachine.add('VERIFY_THE_WORKSPACE_IS_NOT_EMPTY', GetTableObjectCluster(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose' })
            """

################################################################################
#Simple detection state machine with environment confirmation
#
#Robot would detect the object autonomously
#Environment confirmation is needed for this detection
#
#If the target object name/id is empty, then the robot will search all the objects in the field of view
#if the environment confirmation/verification is needed, the target workspace name is used to get the pose of the work space
#in the simple detection, only the nearest object to the camera is detected
#
#the id of the object is not used by the detection state yet, this should be changed
################################################################################

class sm_simple_detection_env(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name', 'target_object_id', 'target_workspace_name'],
                                    output_keys=['target_object','target_object_pose'])

        self.max_retries = 5  # default value for max retries 
        try:
            self.max_retries = rospy.get_param("srs/common/detection_max_retries")
        except Exception, e:
            rospy.INFO('can not read parameter of max retries, use the default value')
        
        self.userdata.target_object=''
        
        self.userdata.target_object_pose=''

        with self:
            smach.StateMachine.add('WORKSPACE_CONFRIMATION', workspace_confirmation(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'target_workspace_name':'target_workspace_name'})
            smach.StateMachine.add('DETECT_OBJECT-simple', detect_object('',self.max_retries),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT-simple', 'no_more_retries':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose' })
            
            
################################################################################
#Assisted detection state machine with environment confirmation
#
#Robot would detect the object autonomously first, and then wait for the use for confirmation and adjustment via bounding box
#Environment confirmation is needed for this detection
#
#If the target object name/id is empty, then the robot will search all the objects in the field of view
#if the environment confirmation/verification is needed, the target workspace name is used to get the pose of the work space
#in the assisted detection, all detected objects will be processed, user will choice the best object from the list
#
#the id of the object is not used by the detection state yet, this should be changed
################################################################################

class sm_assisted_detection_env(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name', 'target_object_id', 'target_workspace_name'],
                                    output_keys=['target_object','target_object_pose'])

        
        self.userdata.target_object_list=''
        self.userdata.new_scan_pose = ''

        with self:
            smach.StateMachine.add('WORKSPACE_CONFRIMATION', workspace_confirmation(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'target_workspace_name':'target_workspace_name'})  
                      
            smach.StateMachine.add('DETECT_OBJECT-assisted', detect_object_assited(),
                    transitions={'succeeded':'USER_INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object_list':'target_object_list' })
            
            smach.StateMachine.add('USER_INTERVENTION', user_intervention_on_detection(),
                    transitions={'succeeded':'succeeded', 'bb_move':'BB_MOVE', 'give_up':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'target_object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose', 'bb_pose':'new_scan_pose', 'target_object_list':'target_object_list'})
            
            smach.StateMachine.add('BB_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'DETECT_OBJECT-assisted', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'base_pose':'new_scan_pose'})    