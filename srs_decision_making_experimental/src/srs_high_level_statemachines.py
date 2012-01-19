#!/usr/bin/python
#################################################################
# \note
#   Project name: srs
# \author
#   Renxi Qiu, email:renxi.qiu@googlemail.com
#
# \date Date of creation: Oct 2011
#################################################################
# ROS imports
from srs_generic_states import *

"""
This file contains high level state machines for decision making.
The actual implementation of the states can be found from 
    srs_generic_states.py 
    or 
    imported from other srs components

Depend on if there is any user client connected and the type of the user client, 
state machines below may switch between semi-autonomous or fully-autonomous mode mode.

If semi_autonomous_mode=True Generic states of user interventions will be loaded for error handling
Otherwise, generic states based on semantic kb for error handling will be loaded 

The default mode is semi_autonomous_mode=False assuming no UI connected to the robot

    sm_approach_pose_assisted()
    #assisted navigation, operator could specify intermediate position for final goal

    sm_detect_asisted_pose_region()
    #detect object, both base pose and region can be adjusted 
    
    sm_pick_object_asisted()
    #pick object with user intervention for error handling
    
    sm_get_object_on_tray(
    #transfer object to tray after pick
    
    sm_enviroment_object_update
    #update the environment object information
    
    
"""

class SRS_StateMachine(smach.StateMachine):
    #This container inherits functionality from smach.StateMachine and adds
    #some auto-generated call that share information among SRS states 
    #
    
    def __init__(self, outcomes, input_keys=[], output_keys=[]):
        super(SRS_StateMachine,self).__init__(outcomes, input_keys, output_keys)
    
    def customised_initial (self, name_of_the_state):
        #initialise userdata for goal
        self.userdata.current_sub_task_name=name_of_the_state
        
        #add necessary cbs
        self.register_termination_cb (self.termination_cb, [])
        self.register_transition_cb(self.transition_cb, [])
        self.register_start_cb(self.start_cb, [])

    def start_cb(self, userdata, intial_state):
        global current_task_info
        _feedback=xmsg.ExecutionFeedback()
        _feedback.current_state = userdata.current_sub_task_name + ": started"
        _feedback.solution_required = False
        _feedback.exceptional_case_id = 0
        current_task_info._srs_as._as.publish_feedback(_feedback)
        rospy.sleep(1)
    
    def transition_cb (self, userdata, active_states):
        pass
                      
    def termination_cb (self, userdata, active_states, outcome ):
        #update the task execution status of the last state / state machine container
        global current_task_info
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = userdata.current_sub_task_name
        last_step_info.outcome = outcome
        current_task_info.last_step_info.append(last_step_info)
        _feedback=xmsg.ExecutionFeedback()
        _feedback.current_state = userdata.current_sub_task_name + ":" + outcome
        _feedback.solution_required = False
        _feedback.exceptional_case_id = 0
        current_task_info._srs_as._as.publish_feedback(_feedback)
        rospy.sleep(1)

####################################################################################
#Navigation state machine
#        
#assisted navigation, operator or semantic KB could specify intermediate position for final goal
#alternatively, use the approach_pose directly, where robot will re-retry by itself
#
####################################################################################
class sm_approach_pose_assisted(SRS_StateMachine):
	
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        self.customised_initial("sm_approach_pose_assisted")
        self.userdata.intermediate_pose=""
        
        with self:
		 	smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
					transitions={'succeeded':'succeeded', 'not_completed':'INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
					remapping={'base_pose':'target_base_pose'})
		
			smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
					transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed','preempted':'preempted'},
					remapping={'semi_autonomous_mode':'semi_autonomous_mode','intermediate_pose':'intermediate_pose'})
		
			smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
					transitions={'succeeded':'APPROACH_POSE', 'not_completed':'INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
					remapping={'base_pose':'intermediate_pose'})
        
################################################################################
#Detection state machine
#
#detection assisted by remote operator or KB, operator could specify region 
#of interest and scanning pose for detection
################################################################################

#detection assisted by remote operator or KB, they specify region of interest for detection
class sm_detect_asisted_region(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object','target_object_pose'])
        self.customised_initial("sm_detect_asisted_region")
        self.userdata.key_region=''
        self.userdata.target_object=''
        self.userdata.target_object_pose=''

        with self:
            smach.StateMachine.add('DETECT_OBJECT-2', detect_object(),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT-2', 'no_more_retries':'INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_pose','key_region':'key_region' })
                    # detection need to be updated to take the key region
                    # remapping={'object_name':'target_object_name', 'object':'target_object_pose', 'key_region':'key_region'})
                    
                    
            smach.StateMachine.add('INTERVENTION', intervention_key_region(),
                    transitions={'retry':'DETECT_OBJECT-2', 'no_more_retry':'not_completed','failed':'failed', 'preempted':'preempted'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','key_region':'key_region'})
        
                                   

#detection assisted by remote operator or KB, operator could specify region of interest and scanning pose for detection
class sm_detect_asisted_pose_region(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object','target_object_pose'])
        self.customised_initial("sm_detect_asisted_pose_region")
        self.userdata.intermediate_pose=""
            
      
        with self:
            smach.StateMachine.add('DETECT_OBJECT-1', sm_detect_asisted_region(),
                    transitions={'succeeded':'succeeded', 'not_completed':'INTERVENTION', 'failed':'failed','preempted':'preempted'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object':'target_object','target_object_pose':'target_object_pose' })
                
            smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
                    transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed','preempted':'preempted'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'intermediate_pose':'intermediate_pose'})
                
            smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'DETECT_OBJECT-1', 'not_completed':'INTERVENTION', 'failed':'failed','preempted':'preempted'},
                    remapping={'base_pose':'intermediate_pose'})    
            

################################################################################
#grasp state machine
#
#grasp assisted by remote operator or KB
################################################################################
class sm_pick_object_asisted(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object', 'target_object_old_pose', 'grasp_categorisation'])
        
        self.customised_initial("sm_pick_object_asisted")
        self.userdata.grasp_categorisation=""
        self.userdata.target_object_old_pose=""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', sm_detect_asisted_pose_region(),
                    transitions={'succeeded':'SELECT_GRASP', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object':'target_object', 'target_object_pose':'target_object_old_pose'})
    
            smach.StateMachine.add('SELECT_GRASP', select_grasp(),
                    transitions={'succeeded':'GRASP_GENERAL', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_categorisation':'grasp_categorisation', 'object':'target_object'})
                
            smach.StateMachine.add('GRASP_GENERAL', grasp_general(max_retries = 5),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retries':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'grasp_categorisation':'grasp_categorisation'})
 
 
################################################################################
#transfer object to tray state machine
#
#
################################################################################
class sm_transfer_object_to_tray(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['grasp_categorisation'])
        
        self.customised_initial("sm_get_object_on_tray")
        self.userdata.post_table_pos=""
    
        with self:
            smach.StateMachine.add("SELECT_POST_TABLE_POS", select_post_table_pose(),
                transitions={'succeeded':'MOVE_TO_POST_TABLE_POS','failed':'failed','preempted':'preempted'},
                remapping={'post_table_pos': 'post_table_pos'})               
            smach.StateMachine.add('MOVE_TO_POST_TABLE_POS', approach_pose_without_retry(),
                transitions={'succeeded':'PUT_OBJECT_ON_TRAY','failed':'failed','preempted':'preempted'},
                remapping={'base_pose':'post_table_pos'})                              
            smach.StateMachine.add('PUT_OBJECT_ON_TRAY', put_object_on_tray(),
                transitions={'succeeded':'succeeded', 'failed':'failed','preempted':'preempted'},
                remapping={'grasp_categorisation':'grasp_categorisation'})       
            
################################################################################
#environment object update state machine
#
#
################################################################################    

class sm_enviroment_model_update(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_base_pose_list'],
                                    output_keys=['reference_to_map']
                                    )
        self.customised_initial("sm_enviroment_model_update")
        self.userdata.reference_to_map=""
        
        with self:
            smach.StateMachine.add('SELECT_POSE', select_pose(),
                    transitions={'got_to_next_pose':'APPROACH_POSE', 'no_more_pose':'succeeded', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'base_pose':'current_target_base_pose','target_base_pose_list':'target_base_pose_list'})   
            smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
                    transitions={'succeeded':'UPDATE_ENVIROMENT', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'base_pose':'target_base_pose'})     
            smach.StateMachine.add('UPDATE_ENVIROMENT', update_env_model(),
                    transitions={'succeeded':'SELECT_POSE', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'reference_to_map':'reference_to_map'})   

            
class sm_enviroment_object_update(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name_list','scan_pose_list'],
                                    output_keys=['target_object_pose_list']
                                    )
        self.customised_initial("sm_enviroment_object_update")
        self.userdata.reference_to_map=""
        self.userdata.target_object_pose=""
        
        with self:
            
            smach.StateMachine.add('MOVE_AND_UPDATE_ENVIROMENT', sm_enviroment_model_update(),
                    transitions={'succeeded':'VERIFY_OBJECT', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'scan_pose_list':'scan_pose_list','reference_to_map':'reference_to_map'})     
            
            smach.StateMachine.add('VERIFY_OBJECT', verify_object(),
                    transitions={'object_verified':'succeeded', 'no_object_verified':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'reference_to_map':'reference_to_map','object_name_list':'target_object_name_list','updated_object_list':'target_object_pose_list'})      
            
################################################################################
#verify an object update at a single position
#
#
################################################################################    

            
class sm_enviroment_object_verification_simple(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name', 'target_object_hh_id', 'target_base_pose', 'target_object_pose'],
                                    output_keys=['verified_target_object_pose']
                                    )
        self.customised_initial("sm_enviroment_object_verification")
        
        with self:
            smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
                    transitions={'succeeded':'UPDATE_ENVIROMENT', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'base_pose':'target_base_pose'})     
            smach.StateMachine.add('UPDATE_ENVIROMENT', update_env_model(),
                    transitions={'succeeded':'VERIFY_OBJECT', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'})  
            smach.StateMachine.add('VERIFY_OBJECT', object_verification_simple(),
                    transitions={'object_verified':'succeeded', 'no_object_verified':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'target_object_name':'target_object_name',
                               'target_object_hh_id':'target_object_hh_id',
                               'target_object_pose':'target_object_pose'
                               })           
            
"""
OLD STATE MACHINES
"""
"""         
#detection assisted by remote operator or semantic KB, it could adjust the scanning pose with interaction
class sm_detect_asisted_pose(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'time_out', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        self.customised_initial("sm_detect_asisted_pose")
        self.userdata.intermediate_pose=""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', detect_object(),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retries':'INTERVENTION', 'failed':'failed', 'preempted':'preempted' },
                    remapping={'object_name':'target_object_name', 'object':'target_object_pose'})
                
            smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
                    transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed','preempted':'preempted'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','intermediate_pose':'intermediate_pose'})
                
            smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'DETECT_OBJECT', 'not_completed':'INTERVENTION', 'failed':'failed', 'preempted':'preempted', 'time_out':'time_out'},
                    remapping={'base_pose':'intermediate_pose'})  
 
class sm_deliver_object(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_base_pose','semi_autonomous_mode'])        
        self.customised_initial("sm_deliver_object")
    
        with self:
            smach.StateMachine.add('MOVE_TO_ORDER', sm_approach_pose_assisted(),
                    transitions={'succeeded':'DELIVER_OBJECT', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted', 'stopped':'stopped'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_base_pose':'target_base_pose'})
                                                  
            smach.StateMachine.add('DELIVER_OBJECT', deliver_object(),
                    transitions={'succeeded':'succeeded', 
                                 'retry':'not_completed',
                                'failed':'failed','preempted':'preempted', 'stopped':'stopped'})         
            
              
            
class charging(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_base_pose','semi_autonomous_mode'])        
        self.customised_initial("go_to_charging_station")
    
        with self:
            smach.StateMachine.add('PREPARE_ROBOT', prepare_robot(),
                    transitions={'succeeded':'MOVE_TO_CHARGING_STATION', 'failed':'failed','preempted':'preempted', 'stopped':'stopped'}
                    )
                                                  
            smach.StateMachine.add('MOVE_TO_CHARGING_STATION', approach_pose_without_retry(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted', 'stopped':'stopped'},
                    remapping={'base_pose':'target_base_pose'})
            
            

class sm_open_door(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'not_completed', 'failed'])
        
        self.customised_initial("sm_open_door")
        self.userdata.door_identifier="ipa_logo"
        
        with self:
            smach.StateMachine.add('DETECT_DOOR', detect_object(),
                transitions={'succeeded':'OPEN_DOOR', 'retry':'DETECT_DOOR', 'no_more_retries':'not_completed', 'failed':'failed'},
                remapping={'object_name':'door_identifier', 'object':'target_object_pose'})

            smach.StateMachine.add('OPEN_DOOR', open_door(),
                transitions={'succeeded':'succeeded', 'failed':'failed'})    
        
"""           
