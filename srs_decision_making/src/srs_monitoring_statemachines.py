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


"""
This file contains concurrent state machines which provide parallel interruption checking during the operation.
"""

#checking termination request for operation, the state will be terminated if:
# 1  the current operation is stoppable
# 2  and a termination request such as stop or customised_preempty is received or the main operation is completed (in this case, a preempty is triggered by the main operation)
class state_checking_during_operation (smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes =['stopped', 'customised_preempted', 'paused', 'preempted'])
        #self.state_checking_outcome = 'preempted'  #default outcome
    
    def execute (self, userdata):
        global current_task_info
        self.state_checking_outcome = 'preempted'  #reset the outcome to default
        _feedback=xmsg.ExecutionFeedback()
        
        while (not self.preempt_requested()):

            #rospy.sleep(1)
            time.sleep(1)

            
            #if stop command has been received
            if current_task_info.get_stop_required()==True:

                #update the final outcome to stopped
                self.state_checking_outcome  = 'stopped'
                
                #if the current action can be stopped in the middle, terminate the checking and trigger preempty to the operation state
                #otherwise wait for the main operation which is not stoppable to be completed
                if current_task_info.stopable():
                    #acknowledge the request
                    current_task_info.set_stop_acknowledged(True)
                    try:
                        sss.say([current_task_info.speaking_language['Stop']],False)
                        _feedback.current_state =  "the task has been stopped"
                        _feedback.solution_required = False
                        _feedback.exceptional_case_id = 0
                        current_task_info._srs_as._as.publish_feedback(_feedback)
                    except:
                        print sys.exc_info()
                    return 'stopped'
                
            elif current_task_info.get_pause_required()==True:
                #update the final outcome to stopped
                self.state_checking_outcome  = 'paused'
                try:
                    _feedback.current_state =  "the task has been paused"
                    _feedback.solution_required = False
                    _feedback.exceptional_case_id = 0
                    current_task_info._srs_as._as.publish_feedback(_feedback)
                    sss.say([current_task_info.speaking_language['Pause']],False)
                except:
                    print sys.exc_info()
                return 'paused'
            
            #if another command with higher priority received
            elif current_task_info.get_customised_preempt_required()==True:

                #update the final outcome to customised_preempted
                self.state_checking_outcome  = 'customised_preempted'
                #if the current action can be stopped in the middle, terminate the checking and trigger preempty to the operation state
                #otherwise wait for the main operation which is not stoppable to be completed
                if current_task_info.stopable():
                    try:
                        _feedback.current_state =  "the task has been replaced by another task request with higher priority"
                        _feedback.solution_required = False
                        _feedback.exceptional_case_id = 0
                        current_task_info._srs_as._as.publish_feedback(_feedback)
                        sss.say([current_task_info.speaking_language['Preempt']],False) 
                        #acknowledge the request
                    except:
                        print sys.exc_info()
                    current_task_info.set_customised_preempt_acknowledged(True)
                    return self.state_checking_outcome
                
            #elif rospy.is_shutdown:
            #    return 'preempted' 

        #preempted
        self.service_preempt()
            
        if self.state_checking_outcome == 'stopped':
            try:
                sss.say([current_task_info.speaking_language['Stop']],False)        		
                _feedback.current_state =  "the task has been stopped"
                _feedback.solution_required = False
                _feedback.exceptional_case_id = 0
                current_task_info._srs_as._as.publish_feedback(_feedback)
                current_task_info.set_stop_acknowledged(True)
            except:
                print sys.exc_info()
        if self.state_checking_outcome == 'customised_preempted':
            try:
                _feedback.current_state =  "the task has been replaced by another task request with higher priority"
                _feedback.solution_required = False
                _feedback.exceptional_case_id = 0
                current_task_info._srs_as._as.publish_feedback(_feedback)
                sss.say([current_task_info.speaking_language['Preempt']],False) 
                #acknowledge the request
            except:
                print sys.exc_info()
                current_task_info.set_customised_preempt_acknowledged(True)
        return self.state_checking_outcome

# gets called when ANY child state terminates
def common_child_term_cb(outcome_map):

    #checking if the termination is triggered by the completion of the main function
    #This will pre-empty the state_checking_during_operation state
    if outcome_map['MAIN_OPERATION'] is not None:   
        return True
    
    #termination is triggered by the checking state 
    #stop command received
    if outcome_map['State_Checking_During_Operation'] == 'stopped':      
        return True
    
    #another command with higher priority received
    if outcome_map['State_Checking_During_Operation'] == 'customised_preempted':
        return True
    
    #pause command received
    if outcome_map['State_Checking_During_Operation'] == 'paused':
        return True
    
    #preempty or shutdown command received
    if outcome_map['State_Checking_During_Operation'] == 'preempted':
        return True
    
    # in all other case, just keep running, don't terminate anything
    # There is no another case yet, just for complete
    return False


# gets called when ALL child states are terminated
def common_out_cb(outcome_map):
    
    # Main operation is terminated before completion by checking state
    if outcome_map['MAIN_OPERATION'] == 'preempted' :
    
        #operation terminated by external stop request
        if outcome_map['State_Checking_During_Operation'] == 'stopped':
            return 'stopped'
            
        #pause is required
        elif outcome_map['State_Checking_During_Operation'] == 'paused':
            return 'paused'
        
        else:            
            #operation terminated by external high priority command, ctrl-c or preempty trigger by the client for the same goal
            return 'preempted'
            
    #operation completed by the main operation. The outcome of the main operation is returned accordingly    
    return outcome_map['MAIN_OPERATION'] 
    


###################################################
# creating the concurrence state machine navigation


co_sm_navigation = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_base_pose','semi_autonomous_mode'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_navigation:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_srs_navigation(),
                            remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'})


###################################################
# creating the concurrence state machine detection


co_sm_detection = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name','target_object_id', 'target_workspace_name','semi_autonomous_mode'],
                 output_keys=['target_object','target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_detection:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_srs_detection(),
                            remapping={'target_object_name':'target_object_name',
                                        'target_object_id':'target_object_id',
                                        'target_workspace_name':'target_workspace_name',
                                        'semi_autonomous_mode':'semi_autonomous_mode',
                                        'target_object_pose':'target_object_pose',
                                        'target_object':'target_object'})


###################################################
# creating the concurrence state machine grasp


co_sm_new_grasp = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name','target_object_id','target_object','target_workspace_name','semi_autonomous_mode'],
                 output_keys=['grasp_categorisation','surface_distance'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_new_grasp:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_srs_new_grasp(),
                            remapping={'target_object_name':'target_object_name',
                                        'semi_autonomous_mode':'semi_autonomous_mode',
                                        'target_object_id':'target_object_id',
                                        'target_object':'target_object',
                                        'target_workspace_name':'target_workspace_name',
                                        'grasp_categorisation':'grasp_categorisation',
                                        'surface_distance':'surface_distance'})


###################################################
# creating the concurrence state machine old grasp


co_sm_old_grasp = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name','target_object_id','target_object','semi_autonomous_mode'],
                 output_keys=['grasp_categorisation','surface_distance'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_old_grasp:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_srs_old_grasp(),
                            remapping={'target_object_name':'target_object_name',
                                        'semi_autonomous_mode':'semi_autonomous_mode',
                                        'target_object_id':'target_object_id',
                                        'target_object':'target_object',
                                        'grasp_categorisation':'grasp_categorisation',
                                        'surface_distance':'surface_distance'})

###################################################
# creating the concurrence state machine put object on tray
# this process can be paused but not stoppable until object is on the tray, robot has to put the target  in a stable position

co_sm_transfer_to_tray = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['grasp_categorisation','surface_distance'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_transfer_to_tray:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_srs_put_on_tray(),
                            remapping={'grasp_categorisation':'grasp_categorisation',
                                       'surface_distance':'surface_distance'})


###################################################
# creating the concurrence state machine environment object update


co_sm_enviroment_update = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['scan_pose_list'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_enviroment_update:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_enviroment_update(),
                            remapping={'scan_pose_list':'scan_pose_list'})



