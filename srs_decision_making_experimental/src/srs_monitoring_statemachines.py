#!/usr/bin/python
#################################################################
# \note
#   Project name: srs
# \author
#   Renxi Qiu, email:renxi.qiu@googlemail.com
#
# \date Date of creation: Dec 2011
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
        smach.State.__init__(self , outcomes =['stopped', 'customised_preempted', 'preempted'])
        self.state_checking_outcome = 'preempted'  #default outcome
    
    def execute (self, userdata):
        global current_task_info
        while not self.preempt_requested():
            #if stop command has been received
            if current_task_info._as.stop_required:
                #reset the flag to normal after the request has been received
                current_task_info._as.stop_required = False
                
                #update the final outcome to stopped
                self.state_checking_outcome  = 'stopped'
                
                #if the current action can be stopped in the middle, terminate the checking and trigger preempty to the operation state
                #otherwise wait for the main operation which is not stoppable to be completed
                if current_task_info.stopable():
                    return self.state_checking_outcome
            
            #if another command with higher priority received
            if current_task_info._as.customised_preempt_required:
                #reset the flag to normal after the request has been received
                current_task_info._as.customised_preempt_required = False

                #update the final outcome to customised_preempted
                self.state_checking_outcome  = 'customised_preempted'
                                
                #if the current action can be stopped in the middle, terminate the checking and trigger preempty to the operation state
                #otherwise wait for the main operation which is not stoppable to be completed
                if current_task_info.stopable():
                    return self.state_checking_outcome
            
            #sleep 1 sec and check again
            rospy.sleep(1)
        
        #preempted by system        
        self.service_preempt()
        
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
    
    # in all other case, just keep running, don't terminate anything
    return False


# gets called when ALL child states are terminated
def common_out_cb(outcome_map):
    
    # Main operation is terminated before completion by checking state
    if outcome_map['MAIN_OPERATION'] == 'preempted' :
    
        #operation terminated by external stop request
        if outcome_map['State_Checking_During_Operation'] == 'stopped':
            return 'stopped'
            
        #operation terminated by external high priority command
        if outcome_map['State_Checking_During_Operation'] == 'customised_preempted':
            return 'preempted'
        
    #check if the termination is triggered by time out during pause.
    if outcome_map['MAIN_OPERATION'] == 'time_out':   
        global current_task_info
        if current_task_info._as.pause_required:
            return 'paused'
        else:
            return 'not_completed' 
            
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
            smach.Concurrence.add('MAIN_OPERATION', sm_approach_pose_assisted(),
                            remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'})

"""
class co_sm_navigation(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_base_pose','semi_autonomous_mode'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 #navigation can be stopped at any time
                                  
        with self:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_approach_pose_assisted(),
                            remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'})
"""
###################################################
# creating the concurrence state machine detection


co_sm_detection = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_detection:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_detect_asisted_pose_region(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})

"""
class co_sm_detection(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_pose'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 #detection can be stopped at any time
                                  
        with self:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_detect_asisted_pose_region(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_pose':'target_object_pose'})
"""

###################################################
# creating the concurrence state machine grasp


co_sm_grasp = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_old_pose', 'grasp_catogorisation'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)

with co_sm_grasp:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_pick_object_asisted(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_old_pose':'target_object_old_pose',
                                       'grasp_categorisation':'grasp_categorisation'})
"""
class co_sm_grasp(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name', 'semi_autonomous_mode'],
                 output_keys=['target_object_old_pose', 'grasp_catogorisation'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 #detection can be stopped at any time
                                  
        with self:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_pick_object_asisted(),
                            remapping={'target_object_name':'target_object_name',
                                       'semi_autonomous_mode':'semi_autonomous_mode',
                                       'target_object_old_pose':'target_object_old_pose',
                                       'grasp_categorisation':'grasp_categorisation'})
"""

###################################################
# creating the concurrence state machine put object on tray
# this process can be paused but not stoppable until object is on the tray, robot has to put the target  in a stable position

co_sm_transfer_to_tray = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['grasp_catogorisation'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_transfer_to_tray:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_transfer_object_to_tray(),
                            remapping={'grasp_categorisation':'grasp_categorisation'})

"""
class co_sm_transfer_to_tray(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'not_completed', 'failed', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['grasp_catogorisation'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 #detection can be stopped at any time
                                  
        with self:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_transfer_object_to_tray(),
                            remapping={'grasp_categorisation':'grasp_categorisation'})
"""    
###################################################
# creating the concurrence state machine environment object update


co_sm_enviroment_object_update = smach.Concurrence (outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name_list', 'scan_pose_list'],
                 output_keys=['target_object_pose_list'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_enviroment_object_update:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_enviroment_object_update(),
                            remapping={'target_object_name_list':'target_object_name_list',
                                       'target_object_pose_list':'target_object_pose_list',
                                       'scan_pose_list':'scan_pose_list',})

"""
class co_sm_enviroment_object_update(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['target_object_name_list', 'scan_pose_list'],
                 output_keys=['target_object_pose_list'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                 #detection can be stopped at any time
                                  
        with self:
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())   
            smach.Concurrence.add('MAIN_OPERATION', sm_enviroment_object_update(),
                            remapping={'target_object_name_list':'target_object_name_list',
                                       'target_object_pose_list':'target_object_pose_list',
                                       'scan_pose_list':'scan_pose_list',})
"""
