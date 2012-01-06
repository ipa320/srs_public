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

from srs_monitoring_statemachines import *

"""
This file contains state machines for robot configuration checking during the operation as well as the pause and resume of the generic state

Please note that:
As generic states are the minimal control unit of the DM, therefore the pause and resume of the generic states may not be the same as the pause and resume of the real actions of robot.
and, it will be useful to check the pause is required within the generic state.  

"""

# sub state, happens when the time out of the main operation is trigger by the external pause require    
# the generic states of the main operation should check the pause request on this own. Hence, this state should be avoided for as much as possible 
class state_checking_during_paused (smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes =['resume', 'stopped', 'preempted'])
    
    
    def execute (self, userdata):

        global current_task_info
        
        while not self.preempt_requested():
            #if stop command has been received during the pause
            if current_task_info._as.stop_required:
                #reset the flag to normal
                current_task_info._as.stop_required = False
                #complete the state and return stop
                return 'stopped'
            
            #if another command with higher priority received during the pause
            if current_task_info._as.customised_preempt_required:
                #reset the flag to normal
                current_task_info._as.customised_preempt_required = False
                #complete the state and return customised_preempted
                return 'preempted'
            
            #if task is resumed
            if not current_task_info._as.customised_pause_required:
                #return to last operation
                return 'resume'
                
            #sleep 1 sec and check again
            rospy.sleep(1)
            
        self.service_preempt()
        
        return 'preempted'



class pre_conf(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succeeded', 'failed', 'stopped', 'preempted'])
    
    def execute (self, userdata):    
        pass
    

class co_sm_pre_conf(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                                  
        with self: 
            smach.concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.concurrence.add('MAIN_OPERATION', preconf())
    
        
class post_conf(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succeeded', 'failed', 'stopped', 'preempted'])
    
    def execute (self, userdata):    
        pass
    

class co_sm_post_conf(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
                                  
        with self: 
            smach.concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.concurrence.add('MAIN_OPERATION', post_conf())
    
    
    
    
    

class srs_navigation(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf(),
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('ACTION', co_sm_navigation(),
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf(),
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})


class srs_detection(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf(),
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('ACTION', co_sm_detection(),
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_object_name':'target_object_name','target_object_pose':'target_object_pose' })
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf(),
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})



class srs_grasp(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_old_pose', 'grasp_catogorisation'])
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf(),
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('ACTION', co_sm_grasp(),
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'target_object_name':'target_object_name','semi_autonomous_mode':'semi_autonomous_mode','target_object_old_pose':'target_object_old_pose','grasp_catogorisation':'grasp_catogorisation'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf(),
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})


class srs_put_on_tray(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['grasp_catogorisation'])
        self.userdata.resume_point =""
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf(),
                    transitions={'succeeded':'OPERATION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('OPERATION', co_sm_transfer_to_tray(),
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_MAIN_OPERATION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'grasp_catogorisation':'grasp_catogorisation'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf(),
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_MAIN_OPERATION', state_checking_during_paused(),
                    transitions={'resume':'OPERATION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})


class srs_enviroment_object_update(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf(),
                    transitions={'succeeded':'OPERATION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('OPERATION', co_sm_grasp(),
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_MAIN_OPERATION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'target_object_name':'target_object_name','semi_autonomous_mode':'semi_autonomous_mode', 'target_object_pose':'target_object_pose'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf(),
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'})
        
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_MAIN_OPERATION', state_checking_during_paused(),
                    transitions={'resume':'OPERATION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})








