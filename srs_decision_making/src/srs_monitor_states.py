#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2011 \n
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
# \date Date of creation: Oct 2011
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

from srs_high_level_statemachines import *
from smach import Concurrence

  
class State_Checking_During_Operation (smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes =['stopped', 'customised_preempted', 'preempted'])
    
    
    def execute (self, userdata):
        global current_task_info
        while not self.preempt_requested():
            #if stop command has been received
            if current_task_info._as.stop_required:
                #reset the flag to normal
                current_task_info._as.stop_required = False
                #complete the state and return stop
                return 'stopped'
            
            #if another command with higher priority received
            if current_task_info._as.customised_preempt_required:
                #reset the flag to normal
                current_task_info._as.customised_preempt_required = False
                #complete the state and return customised_preempted
                return 'customised_preempted'
            
            #sleep 1 sec and check again
            rospy.sleep(1)
        
        #preempted by system        
        self.service_preempt()
        
        return 'preempted'

# gets called when ANY child state terminates
# used by stoppable actions such as navigation or detection or grasp
# the action can be stopped in the middle of the operation 
def common_child_term_cb_stoppable(outcome_map):

    #checking if the termination is triggered by the completion of the main function
    #This will pre-empty the state_checking_during_operation state
    if outcome_map['MAIN_OPERATION'] is not None:   
        return True
    
    #termination is triggered by the checking state 
    #stop command received
    if outcome_map['State_Checking_During_Operation'] == 'stopped':      
        return True
    
    #another command with higher priority received
    if outcome_map['State_Check_During_Operation'] == 'customised_preempted':
        return True
    
    # in all other case, just keep running, don't terminate anything
    return False


# gets called when ANY child state terminates
# used by non-stoppable actions such as move object to tray after the object has been grasped
# the action has to be completed, it make no sense to stop in the middle of the operation 
def common_child_term_cb_non_stoppable(outcome_map):

    #checking if the termination is triggered by the completion of the main function 
    if outcome_map['MAIN_OPERATION'] is not None:   
        return True
    # terminate all running states if state checking finished with outcome 'stopped, customised_preempted or paused'
    
    #stop command received
    if outcome_map['State_Check_During_Operation'] == 'stopped':      
        return True
    
    #another command with higher priority received
    if outcome_map['State_Check_During_Operation'] == 'customised_preempted':
        return True
    
    # in all other case, just keep running, don't terminate anything
    return False



# gets called when ALL child states are terminated
def common_out_cb(outcome_map):
    
    
    #checking if the termination is triggered by the completion of the main function or the monitoring state
    if outcome_map['MAIN_OPERATION'] == 'preempted':   
        #operation terminated by external stop request
        if outcome_map['State_Check_During_Operation'] == 'stopped':
            
            return 'stopped'
        
        #operation terminated by external pause request
        if outcome_map['State_Check_During_Operation'] == 'paused':
            return 'paused'
        
        #operation terminated by external preempty request
        if outcome_map['State_Check_During_Operation'] == 'customised_preempted':
            return 'preempted'
        
        #This is unlikely, un expected preempty generated by main operation itself 
        else:
            return 'preempted'
    
    #operation completed by the main operation. The outcome is returned accordingly    
    else:
        return outcome_map['MAIN_OPERATION'] 


###################################################
# creating the concurrence state machine navigation
srs_sm_navigation = Concurrence(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_base_pose','semi_autonomous_mode'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 
with srs_sm_navigation:
    smach.concurrence.add('State_Check_During_Operation', State_Checking_During_Operation())
    
    smach.concurrence.add('MAIN_OPERATION', sm_approach_pose_assisted(),
                            remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'})

###################################################
# creating the concurrence state machine detection
srs_sm_detection = Concurrence(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 
with srs_sm_detection:
    smach.concurrence.add('State_Check_During_Operation', State_Checking_During_Operation())
    
    smach.concurrence.add('MAIN_OPERATION', sm_detect_asisted_pose_region(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})

###################################################
# creating the concurrence state machine grasp
srs_sm_grasp = Concurrence(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_old_pose', 'grasp_catogorisation'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 
with srs_sm_grasp:
    smach.concurrence.add('State_Check_During_Operation', State_Checking_During_Operation())
    
    smach.concurrence.add('MAIN_OPERATION', sm_get_object_on_tray(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})



###################################################
# creating the concurrence state machine put object on tray
srs_sm_grasp = Concurrence(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode', 'grasp_catogorisation'],
                 output_keys=['target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 
with srs_sm_grasp:
    smach.concurrence.add('State_Check_During_Operation', State_Checking_During_Operation())
    
    smach.concurrence.add('MAIN_OPERATION', sm_put_on_tray(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})
    
###################################################
# creating the concurrence state machine environment update
srs_sm_grasp = Concurrence(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 
with srs_sm_grasp:
    smach.concurrence.add('State_Check_During_Operation', State_Checking_During_Operation())
    
    smach.concurrence.add('MAIN_OPERATION', sm_put_on_tray(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})
    













class State_Checking_During_Pause (smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes =['resume', 'stopped', 'customised_preempted', 'preempted'])
    
    
    def execute (self, userdata):
        global current_task_info
        while not self.preempt_requested():
            if current_task_info._as.stop_required:
                return 'stopped'
            if not current_task_info._as.pause_required:
                return 'resume'
            if current_task_info._as.customised_preempt_required:
                return 'customised_preempted'
            rospy.sleep(1)
        self.service_preempt()
        return 'preempted'


