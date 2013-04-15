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
import roslib; roslib.load_manifest('srs_decision_making')
import json
#import states within srs_decision_making
from srs_generic_states import *


#import states machines
from srs_common_high_level_statemachines import * 
from srs_detection_high_level_statemachines import *
from srs_grasp_high_level_statemachines import *

#from generic_grasp_state import *

# This is come from srs_object_verification, the name should be updated to avoid confusion
#from generic_states import *

"""
This file contains high level state machines for decision making.
The actual implementation of the states can be found from 
    srs_generic_states.py 
    or 
    imported from other srs components


    
    
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
        _feedback.json_feedback = ''
        if not current_task_info.json_parameters == '':
            _feedback.json_feedback = self.get_json_feedback(userdata.current_sub_task_name, "started")         
        
        current_task_info._srs_as._as.publish_feedback(_feedback)

        rospy.sleep(1)
    
    def transition_cb (self, userdata, active_states):
        pass
                      
    def termination_cb (self, userdata, active_states, outcome ):
        #update the task execution status of the last state / state machine container
        global current_task_info      
       
        _feedback=xmsg.ExecutionFeedback()
        _feedback.json_feedback = ''
        if not current_task_info.json_parameters == '':
            _feedback.json_feedback = self.get_json_feedback(userdata.current_sub_task_name, outcome)        
        _feedback.current_state = userdata.current_sub_task_name + ":" + outcome
        _feedback.solution_required = False
        _feedback.exceptional_case_id = 0
               
        current_task_info._srs_as._as.publish_feedback(_feedback)
        
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = userdata.current_sub_task_name
        last_step_info.outcome = outcome
        current_task_info.last_step_info.append(last_step_info)
        
        rospy.sleep(1)


    def get_json_feedback (self, name_of_the_action, state_of_the_action):
        
        global current_task_info
        
        step_id = len (current_task_info.last_step_info) 
        
        print step_id
        
        json_feedback_current_action = '"current_action": {"name": "'+ name_of_the_action +'", "state": "' + state_of_the_action + '", "step_id": "'+ str(step_id+1) +'", "target_object": "'+ current_task_info.task_feedback.action_object +'", "parent_object": "'+ current_task_info.task_feedback.action_object_parent +'"'+' }'
        
        json_feedback_last_action =''
        
        json_feedback_feedback = '"feedback": {"lang": "'+ current_task_info.language_set +'", "message": "'+ current_task_info.feedback_messages[name_of_the_action] +'"}'
                     
        json_feedback_task = '"task": {"task_id": "'+ str(current_task_info.task_feedback.task_id) +'", "task_initializer": "'+ current_task_info.task_feedback.task_initializer +'","task_initializer_type": "'+ current_task_info.task_feedback.task_initializer_type +'", "task_name": "'+ current_task_info.task_feedback.task_name +'","task_parameter": "'+ current_task_info.task_feedback.task_parameter +'"}'

        if step_id > 0:
            json_feedback_last_action = '"last_action": {"name": "'+ current_task_info.last_step_info[step_id-1].step_name +'","outcome": "'+ current_task_info.last_step_info[step_id-1].outcome +'", "step_id": '+ str(step_id) +'}'
            return json.dumps ('{' + json_feedback_current_action + ',' + json_feedback_feedback + ',' + json_feedback_last_action + ',' + json_feedback_task + '}')
        
        else:
            return json.dumps ('{' + json_feedback_current_action + ',' + json_feedback_feedback + ',' + json_feedback_task + '}')
        
        

####################################################################################
#Navigation state machine
#        
#assisted navigation, operator or semantic KB could specify intermediate position for final goal
#alternatively, use the approach_pose directly, where robot will re-retry by itself
#
####################################################################################
class sm_srs_navigation(SRS_StateMachine):
	
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        self.customised_initial("sm_srs_navigation")
        
        with self:
		 	smach.StateMachine.add('SRS_NAVIGATION', sm_approach_pose_assisted(),
					transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
					remapping={'target_base_pose':'target_base_pose', 'semi_autonomous_mode':'semi_autonomous_mode'})
                                    
####################################################################################
#Detection state machine
#        
#assisted or simple with or without environment confirmation
#
####################################################################################
#detection assisted by remote operator or KB, operator could specify region of interest and scanning pose for detection

class sm_srs_detection(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name','target_object_id', 'target_workspace_name','semi_autonomous_mode'],
                                    output_keys=['target_object','target_object_pose'])
        self.customised_initial("sm_srs_detection") 
        self.detection_type = 'simple'
        self.enviroment_confimation_required = False  
        try:
            self.detection_type = rospy.get_param("srs/detection_type")
            self.enviroment_confimation_required  = rospy.get_param("srs/enviroment_confirmation")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default value for detection and environment update")        
      
        # states related to detection
        if self.detection_type.lower() == 'assisted':
            if self.enviroment_confimation_required == 'False':
                #assisted detection with environment confirmation
                with self:
                    smach.StateMachine.add('SM_DETECTION-assisted', sm_assisted_detection(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'target_object_id':'target_object_id',
                                              'target_workspace_name':'target_workspace_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                               'target_object_pose':'target_object_pose',
                                               'target_object':'target_object'})
            else:
                #assisted detection without environment confirmation
                with self:
                    smach.StateMachine.add('SM_DETECTION-assisted-env', sm_assisted_detection_env(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'target_object_id':'target_object_id',
                                              'target_workspace_name':'target_workspace_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                               'target_object_pose':'target_object_pose',
                                               'target_object':'target_object'})

        else:
            if self.enviroment_confimation_required == 'False':
                #assisted detection with environment confirmation
                with self:
                    smach.StateMachine.add('SM_DETECTION-simple', sm_simple_detection(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'target_object_id':'target_object_id',
                                              'target_workspace_name':'target_workspace_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                               'target_object_pose':'target_object_pose',
                                               'target_object':'target_object' })            
                
            else:
                #assisted detection without environment confirmation
                with self:
                    smach.StateMachine.add('SM_DETECTION-simple-env', sm_simple_detection_env(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'target_object_id':'target_object_id',
                                              'target_workspace_name':'target_workspace_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                               'target_object_pose':'target_object_pose',
                                               'target_object':'target_object' })
            
            



####################################################################################
#Grasp state machine
#        
#assisted planned or simple grasp (The grasp does not involve detection)
#
#
####################################################################################


class sm_srs_new_grasp(SRS_StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_object_name','target_object_id','target_object','target_workspace_name','semi_autonomous_mode'],
            output_keys=['grasp_categorisation', 'surface_distance'])
        
        self.customised_initial("sm_srs_grasp")
        
        #environment switches for development purpose. Should be assisted by default when the project is completed  
        self.grasp_type = 'assisted'
        try:
            self.grasp_type = rospy.get_param("/srs/grasping_type")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default value for grasp")
        

        if self.grasp_type.lower() == 'planned':
            #assisted grasp
            with self:
                smach.StateMachine.add('SM_GRASP-planned', sm_srs_grasp_planned(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                              'target_object_id':'target_object_id',
                                              'target_object':'target_object',
                                              'target_workspace_name':'target_workspace_name',
                                              'grasp_categorisation':'grasp_categorisation', 
                                              'surface_distance':'surface_distance'})
        
        if self.grasp_type.lower() == 'assisted':
            #assisted grasp
            with self:
                smach.StateMachine.add('SM_GRASP-assisted', sm_srs_grasp_assisted(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode', 	
                                              'target_object_id':'target_object_id',
                                              'target_object':'target_object',
                                              'target_workspace_name':'target_workspace_name',
                                              'grasp_categorisation':'grasp_categorisation', 
                                              'surface_distance':'surface_distance'})
        else:
            #simple grasp    
            with self:
                smach.StateMachine.add('SM_GRASP-simple', sm_srs_grasp_simple(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                              'target_object_id':'target_object_id',
                                              'target_object':'target_object',
                                              'target_workspace_name':'target_workspace_name',
                                              'grasp_categorisation':'grasp_categorisation', 
                                              'surface_distance':'surface_distance'})

################################################################################
#Old grasp for backward compatibility (the grasp include the detection)
#
#
################################################################################
class sm_srs_old_grasp(SRS_StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_object_name','target_object_id','target_object','semi_autonomous_mode'],
            output_keys=['grasp_categorisation','surface_distance'])
        
        self.customised_initial("sm_srs_grasp")
        
        with self:
                 smach.StateMachine.add('SM_GRASP-old', sm_pick_object_asisted(),
                                   transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                                   remapping={'target_object_name':'target_object_name',
                                              'semi_autonomous_mode':'semi_autonomous_mode',
                                              'target_object_id':'target_object_id',
                                              'target_object':'target_object',
                                              'grasp_categorisation':'grasp_categorisation', 
                                              'surface_distance':'surface_distance'})
################################################################################
#Transfer object to tray state machine
#
#
################################################################################
class sm_srs_put_on_tray(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
            input_keys=['grasp_categorisation','surface_distance'])
        
        self.customised_initial("sm_put_object_on_tray")
        self.userdata.post_table_pos=""
    
        with self:
            smach.StateMachine.add("PUT_ON_TRAY", sm_transfer_object_to_tray(),
                transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                remapping={'grasp_categorisation': 'grasp_categorisation','surface_distance':'surface_distance'})                  
            
################################################################################
#3D environment update state machine
#
#
################################################################################    
          
class sm_enviroment_update(SRS_StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['scan_pose_list'],
                                    )
        self.customised_initial("sm_enviroment_update")
        
        with self:
            
            smach.StateMachine.add('UPDATE_ENVIROMENT', sm_enviroment_model_update(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'scan_pose_list':'scan_pose_list'})     
               
  
