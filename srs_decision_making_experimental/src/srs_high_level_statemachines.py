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
    imported from other srs component

Depend on if there is any user client connected and the type of the user client, 
state machines below may switch between semi-autonomous or fully-autonomous mode mode.

If semi_autonomous_mode=True Generic states of user interventions will be loaded for error handling
Otherwise, generic states based on semantic kb for error handling will be loaded 

The default mode is semi_autonomous_mode=False assuming no UI connected to the robot

    sm_approach_pose_assisted()
    #assisted navigation, operator could specify intermediate position for final goal
   
    sm_detect_asisted_pose()
    #operator or KB could adjust the scanning pose 
    
    sm_detect_asisted_region()
    #operator or KB could specify region of interest
    
    sm_open_door()
    #open door 
  
    sm_pick_object_basic()
    #basic routine for pick object

    sm_detect_asisted_pose_region()
    #detect object, both base pose and region can be adjusted 
    
    sm_pick_object_asisted()
    #pick object with user intervention for error handling
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
        pass
    
    def transition_cb (self, userdata, active_states):
        pass
                      
    def termination_cb (self, userdata, active_states, outcome ):
        #update the task execution status of the last state / state machine container
        global current_task_info
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = userdata.current_sub_task_name
        last_step_info.outcome = outcome
        current_task_info.last_step_info.append(last_step_info)

        
#assisted navigation, operator or semantic KB could specify intermediate position for final goal
#alternatively, use the approach_pose directly, where robot will re-retry by itself
class sm_approach_pose_assisted(SRS_StateMachine):
	
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        self.customised_initial("sm_approach_pose_assisted")
        self.userdata.intermediate_pose=""
        
        with self:
		 	smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
					transitions={'succeeded':'succeeded', 'failed':'INTERVENTION'},
					remapping={'base_pose':'target_base_pose'})
		
			smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
					transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed'},
					remapping={'semi_autonomous_mode':'semi_autonomous_mode','intermediate_pose':'intermediate_pose'})
		
			smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
					transitions={'succeeded':'APPROACH_POSE', 'failed':'INTERVENTION'},
					remapping={'base_pose':'intermediate_pose'})
        


class sm_enviroment_model_update(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed'],
                                    input_keys=['target_base_pose'],
                                    output_keys=['reference_to_map']
                                    )
        self.customised_initial("sm_enviroment_model_update")
        self.userdata.reference_to_map=""
        
        with self:
            smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
                    transitions={'succeeded':'UPDATE_ENVIROMENT', 'failed':'failed'},
                    remapping={'base_pose':'target_base_pose'})     
            
            smach.StateMachine.add('UPDATE_ENVIROMENT', update_env_model(),
                    transitions={'succeeded':'succeeded', 'failed':'failed'},
                    remapping={'reference_to_map':'reference_to_map'})   

            
class sm_enviroment_object_update(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','target_base_pose'],
                                    output_keys=['target_object_pose']
                                    )
        self.customised_initial("sm_enviroment_object_update")
        self.userdata.reference_to_map=""
        self.userdata.target_object_pose=""
        
        with self:
            
            smach.StateMachine.add('MOVE_AND_UPDATE_ENVIROMENT', sm_enviroment_model_update(),
                    transitions={'succeeded':'VERIFY_OBJECT', 'failed':'failed'},
                    remapping={'target_base_pose':'target_base_pose','reference_to_map':'reference_to_map'})     
            
            smach.StateMachine.add('VERIFY_OBJECT', verify_object(),
                    transitions={'object_verified':'succeeded', 'no_object_verified':'not_completed', 'failed':'failed'},
                    remapping={'reference_to_map':'reference_to_map','object_name':'target_object_name','updated_object':'target_object_pose'})   

            
#detection assisted by remote operator or semantic KB, it could adjust the scanning pose with interaction
class sm_detect_asisted_pose(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        self.customised_initial("sm_detect_asisted_pose")
        self.userdata.intermediate_pose=""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', detect_object(),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retries':'INTERVENTION', 'failed':'failed'},
                    remapping={'object_name':'target_object_name', 'object':'target_object_pose'})
                
            smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
                    transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','intermediate_pose':'intermediate_pose'})
                
            smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'DETECT_OBJECT', 'failed':'INTERVENTION'},
                    remapping={'base_pose':'intermediate_pose'})  
            

#detection assisted by remote operator or KB, they specify region of interest for detection
class sm_detect_asisted_region(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        self.customised_initial("sm_detect_asisted_region")
        self.userdata.key_region=""

        with self:
            smach.StateMachine.add('DETECT_OBJECT', detect_object(),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retries':'INTERVENTION', 'failed':'failed'},
                    remapping={'object_name':'target_object_name', 'object':'target_object_pose'})
                    # detection need to be updated to take the key region
                    # remapping={'object_name':'target_object_name', 'object':'target_object_pose', 'key_region':'key_region'})
                    
                    
            smach.StateMachine.add('INTERVENTION', intervention_key_region(),
                    transitions={'retry':'DETECT_OBJECT', 'no_more_retry':'not_completed','failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','key_region':'key_region'})
        
                                   

#detection assisted by remote operator or KB, operator could specify region of interest and scanning pose for detection
class sm_detect_asisted_pose_region(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        self.customised_initial("sm_detect_asisted_pose_region")
        self.userdata.intermediate_pose=""
            
      
        with self:
            smach.StateMachine.add('DETECT_OBJECT', sm_detect_asisted_region(),
                    transitions={'succeeded':'succeeded', 'not_completed':'INTERVENTION', 'failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object_pose':'target_object_pose' })
                
            smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
                    transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'intermediate_pose':'intermediate_pose'})
                
            smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'DETECT_OBJECT', 'failed':'INTERVENTION'},
                    remapping={'base_pose':'intermediate_pose'})    

                                          
class sm_pick_object_basic(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        self.customised_initial("sm_pick_object_basic")
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', detect_object(),
                transitions={'succeeded':'SELECT_GRASP', 'retry':'DETECT_OBJECT', 'no_more_retries':'object_not_picked', 'failed':'failed'},
                remapping={'object_name':'target_object_name', 'object':'target_object_pose'})
            # object_name is the input key and object is the output key of the detect_object 

            smach.StateMachine.add('SELECT_GRASP', select_grasp(),
                transitions={'top':'GRASP_TOP', 'side':'GRASP_SIDE', 'failed':'failed'})
            
            smach.StateMachine.add('GRASP_SIDE', grasp_side(),
                transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retry':'not_completed', 'failed':'failed'},
                remapping={'target_object_pose':'object'})
            # object is the input key of the grasp
                
            smach.StateMachine.add('GRASP_TOP', grasp_top(),
                transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'no_more_retry':'not_completed', 'failed':'failed'},
                remapping={'target_object_pose':'object'})
            # object is the input key of the grasp
            
       
         
                     
        
class sm_pick_object_asisted(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        
        self.customised_initial("sm_pick_object_asisted")
        self.userdata.grasp_conf=""
        self.userdata.target_object_pose=""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', sm_detect_asisted_pose_region(),
                    transitions={'succeeded':'SELECT_GRASP', 'not_completed':'not_completed', 'failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object_pose':'target_object_pose' })
    
            smach.StateMachine.add('SELECT_GRASP', intervention_grasp_selection(),
                    transitions={'retry':'GRASP_GENERAL', 'no_more_retry':'not_completed','failed':'failed'},
                    remapping={'semi_autonomous_mode':'semi_autonomous_mode','grasp_conf':'grasp_conf'})
                
            smach.StateMachine.add('GRASP_GENERAL', grasp_general(),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT', 'failed':'failed'},
                    remapping={'object':'target_object_pose', 'grasp_conf':'grasp_conf'})
                # object is the input key of the grasp
                

class sm_pick_object_ipa(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['object_picked_top', 'object_picked_side', 'not_completed', 'failed'],
            input_keys=['target_object_name','semi_autonomous_mode'],
            output_keys=['target_object_pose'])
        self.customised_initial("sm_pick_object_ipa")
        self.userdata.target_object_pose=""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT', sm_detect_asisted_pose(),
                transitions={'succeeded':'SELECT_GRASP', 
                            'not_completed':'not_completed', 
                            'failed':'failed'},
                remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object_pose':'target_object_pose' })


            smach.StateMachine.add('SELECT_GRASP', select_grasp(),
                transitions={'top':'GRASP_TOP', 
                            'side':'GRASP_SIDE', 
                            'failed':'failed'},
                remapping={'object':'target_object_pose'})
            
            smach.StateMachine.add('GRASP_SIDE', grasp_side(max_retries = 5),
                transitions={'succeeded':'object_picked_side', 
                            'retry':'DETECT_OBJECT', 
                            'no_more_retries':'not_completed', 
                            'failed':'failed'},
                remapping={'object':'target_object_pose'})
            
            smach.StateMachine.add('GRASP_TOP', grasp_top(max_retries = 5),
                transitions={'succeeded':'object_picked_top', 
                            'retry':'DETECT_OBJECT', 
                            'no_more_retries':'not_completed', 
                            'failed':'failed'},
                remapping={'object':'target_object_pose'})
 
 
class sm_get_object_on_tray(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object_pose'])
        
        self.customised_initial("sm_get_object_on_tray")
        self.userdata.target_object_pose=""
    
        with self:
            smach.StateMachine.add('SM_PICK_OBJECT', sm_pick_object_ipa(),
                transitions={'object_picked_side':'MOVE_TO_POST_TABLE_SIDE', 
                            'object_picked_top':'MOVE_TO_POST_TABLE_TOP',  
                            'not_completed':'not_completed', 
                            'failed':'failed'},
                remapping={'semi_autonomous_mode':'semi_autonomous_mode', 'target_object_name':'target_object_name','target_object_pose':'object_pose' })

                            
            smach.StateMachine.add('MOVE_TO_POST_TABLE_SIDE', approach_pose_without_retry("kitchen"),
                transitions={'succeeded':'PUT_OBJECT_ON_TRAY_SIDE', 
                            'failed':'failed'})
    
            smach.StateMachine.add('MOVE_TO_POST_TABLE_TOP', approach_pose_without_retry("kitchen"),
                transitions={'succeeded':'PUT_OBJECT_ON_TRAY_TOP', 
                            'failed':'failed'})
    
            smach.StateMachine.add('PUT_OBJECT_ON_TRAY_SIDE', put_object_on_tray_side(),
                    transitions={'succeeded':'succeeded', 
                                'failed':'failed'})
                                                  
            smach.StateMachine.add('PUT_OBJECT_ON_TRAY_TOP', put_object_on_tray_top(),
                    transitions={'succeeded':'succeeded', 
                                'failed':'failed'})           
            
     

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
        
            
