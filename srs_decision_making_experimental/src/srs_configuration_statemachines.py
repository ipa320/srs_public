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
from robot_configuration import *

"""
This file contains state machines for robot configuration checking during the operation 
The pause and resume generic state are also included in the file

The actual robot configuration pre post condition are imported from robot_configuration.py 

Please note that:
As minimal control unit of the DM is generic state, therefore the pause and resume of the generic states may not be the same as the pause and resume of the real actions of robot.
and, it will be useful to check the pause within the generic state to avoid unnecessary time out.   

"""

# Checking during robot is paused
class state_checking_during_paused (smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes =['resume', 'stopped', 'preempted'])
    
    
    def execute (self, userdata):

        global current_task_info
        
        
        while not self.preempt_requested():
            #if stop command has been received during the pause
            if current_task_info.get_stop_required():
                #reset the flag to normal
                current_task_info.set_stop_acknowledged(True)
                try:
                    sss.say(["I am stopping."],False)
                except:
                    print sys.exc_info()
                    
                #complete the state and return stop
                return 'stopped'
            
            #if another command with higher priority received during the pause
            elif current_task_info.get_customised_preempt_required():
                #reset the flag to normal
                current_task_info.set_customised_preempt_acknowledged(True)
                #complete the state and return customised_preempted
                return 'preempted'
            
            #if task is resumed
            elif not current_task_info.get_pause_required():
                #return to last operation
                try:
                    sss.say(["I am resuming the task."],False)
                except:
                    print sys.exc_info()                
                return 'resume'
                        
            #elif rospy.is_shutdown:
            #    return 'preempted' 
            
            #sleep 1 sec and check again
            rospy.sleep(1)
            
        self.service_preempt()
        
        return 'preempted'


def robot_configuration(parent, action_name, action_stage):
    
    global current_task_info
    global component_list
    global robot_config
    global robot_config_need_no_action
    
    handles = list()
    
    if action_name == 'navigation':
        if current_task_info.object_on_tray: 
            if current_task_info.object_in_hand:
                action_name = 'navigation_object_on_tray'
            else:
                action_name = 'navigation_object_on_tray_and_sdh'
        else:
            if current_task_info.object_in_hand:
                action_name = 'navigation_object_in_sdh'
            else:
                action_name = 'navigation_no_object'
                
    try:
        #bring robot into the pre-configuration state     
        if action_stage == 'pre-config':
            #initial the sss handles
            for index in range(len(component_list)):
                if robot_config_pre[action_name][component_list[index]] in robot_config_need_no_action: 
                    handles.append(None)
                else:
                    handles.append(sss.move(component_list[index], robot_config_pre[action_name][component_list[index]], False))
                    
        #bring robot into the post-configuration state     
        if action_stage == 'post-config':
            #initial the sss handles
            for index in range(len(component_list)):
                if robot_config_post[action_name][component_list[index]] in robot_config_need_no_action: 
                    handles.append(None)
                else:
                    handles.append(sss.move(component_list[index], robot_config_post[action_name][component_list[index]], False))                
                    
    except KeyError:
        print("dictionary key is not found in the set of existing keys")    
        return failed
    except IndexError:
        print("Index is out of range")
        return failed
    except:
        print("unexpected error during %s and %s",action_name,action_stage)
        return failed
        
    #wait for action to finish
    for index in range(len(component_list)):
        if handles[index] != None:
            if parent.preempt_requested():  
                parent.service_preempt()    
                return 'preempted'
            else:                
                handles[index].wait()
                ###########################################################################
                #TO DO 
                #need check the state of the handles. return failed after the handles fails.
                #
                ############################################################################

    return 'succeeded'
    
                    
            
             


class pre_conf(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succeeded', 'failed', 'preempted'], input_keys=['action_name'])
    
    def execute (self, userdata):    
        return robot_configuration(self, userdata.action_name, 'pre-config')
    
        
class post_conf(smach.State):
    def __init__(self):
        smach.State.__init__(self , outcomes=['succeeded', 'failed',  'preempted'], input_keys=['action_name'])
    
    def execute (self, userdata):    
        return robot_configuration(self, userdata.action_name, 'post-config')


co_sm_pre_conf = smach.Concurrence (outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['action_name'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_pre_conf: 
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.Concurrence.add('MAIN_OPERATION', pre_conf(),
                                  remapping={'action_name':'action_name'})
            
co_sm_post_conf = smach.Concurrence  (outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 input_keys=['action_name'],
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
with co_sm_post_conf: 
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.Concurrence.add('MAIN_OPERATION', post_conf(),
                                  remapping={'action_name':'action_name'})

"""
#It is impossible to reach paused state as the sss used in pre/post conf checking the pause by itself, and will never return time-out 
class co_sm_pre_conf(smach.Concurrence):
    def __init__(self, action_name=''):
        smach.Concurrence.__init__(outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
        self.action_name=action_name
                                  
        with self: 
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.Concurrence.add('MAIN_OPERATION', preconf(self.action_name))
    
#It is impossible to reach paused state as the sss used in pre/post conf checking the pause by itself, and will never return time-out 
class co_sm_post_conf(smach.Concurrence):
    def __init__(self, action_name=''):
        smach.Concurrence.__init__(outcomes=['succeeded', 'failed', 'stopped', 'preempted', 'paused'],
                 default_outcome='failed',
                 child_termination_cb = common_child_term_cb,
                 outcome_cb = common_out_cb)
        self.action_name=action_name
                                  
        with self: 
            smach.Concurrence.add('State_Checking_During_Operation', state_checking_during_operation())
            smach.Concurrence.add('MAIN_OPERATION', post_conf(self.action_name))
"""            


"""
def add_common_states(parent):
    with parent:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})    
"""   


class srs_navigation(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        #self.action_name = 'navigation'
        self.userdata.action_name = 'navigation'
        #add_common_states(self)
        
        with self:              

            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('ACTION', co_sm_navigation,
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_base_pose':'target_base_pose'}) 
            
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
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
        self.userdata.action_name = 'detection'
        #add_common_states(self)
        
        with self:
                
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})        
            
            smach.StateMachine.add('ACTION', co_sm_detection,
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','target_object_name':'target_object_name','target_object_pose':'target_object_pose'  })
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
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
                                    output_keys=['grasp_categorisation', 'target_object_old_pose'])
        self.userdata.action_name = 'grasp'
        #add_common_states(self)
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('ACTION', co_sm_grasp,
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'target_object_name':'target_object_name','semi_autonomous_mode':'semi_autonomous_mode','target_object_old_pose':'target_object_old_pose','grasp_categorisation':'grasp_categorisation'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})    


class srs_put_on_tray(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['grasp_categorisation'])
        self.userdata.action_name = 'put_on_tray'
        #add_common_states(self)
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('ACTION', co_sm_transfer_to_tray,
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'grasp_categorisation':'grasp_categorisation'})

        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})    

class srs_enviroment_object_update(smach.StateMachine):
    
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name_list', 'scan_pose_list'],
                                    output_keys=['target_object_pose_list'])
        self.userdata.action_name = 'enviroment_update'
        #add_common_states(self)
        
        with self:
            smach.StateMachine.add('PRE_CONFIG', co_sm_pre_conf,
                    transitions={'succeeded':'ACTION', 'paused':'PAUSED_DURING_PRE_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})

            smach.StateMachine.add('ACTION', co_sm_enviroment_object_update,
                    transitions={'succeeded':'POST_CONFIG', 'not_completed':'not_completed', 'paused':'PAUSED_DURING_ACTION', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'target_object_name_list':'target_object_name_list', 'target_object_pose_list':'target_object_pose_list', 'scan_pose_list':'scan_pose_list'})
        
            smach.StateMachine.add('POST_CONFIG', co_sm_post_conf,
                    transitions={'succeeded':'succeeded', 'paused':'PAUSED_DURING_POST_CONFIG', 'failed':'failed', 'preempted':'preempted', 'stopped':'stopped'},
                    remapping={'action_name':'action_name'})
            
            smach.StateMachine.add('PAUSED_DURING_PRE_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'PRE_CONFIG','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_ACTION', state_checking_during_paused(),
                    transitions={'resume':'ACTION','preempted':'preempted', 'stopped':'stopped'})
            
            smach.StateMachine.add('PAUSED_DURING_POST_CONFIG', state_checking_during_paused(),
                    transitions={'resume':'POST_CONFIG','preempted':'preempted', 'stopped':'stopped'})    







