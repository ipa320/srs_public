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

#import states within srs_decision_making
from srs_generic_states import *


#import states from srs_states package
from mapping_states import *
from navigation_states import *
from detection_states  import *
from simple_grasp_states import *

#from generic_grasp_state import *

# This is come from srs_object_verification, the name should be updated to avoid confusion
#from generic_states import *

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
    
    sm_get_object_on_tray()
    #transfer object to tray after pick
    
    sm_enviroment_object_update
    #update the environment object information
    
    
    
"""

####################################################################################
#Navigation state machine
#        
#assisted navigation, operator or semantic KB could specify intermediate position for final goal
#alternatively, use the approach_pose directly, where robot will re-retry by itself
#
####################################################################################
class sm_approach_pose_assisted(smach.StateMachine):
	
    def __init__(self):    
        smach.StateMachine.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_base_pose','semi_autonomous_mode'])
        self.userdata.intermediate_pose=""
        
        with self:
            smach.StateMachine.add('APPROACH_POSE', approach_pose_without_retry(),
					transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
					remapping={'base_pose':'target_base_pose'})
            
            smach.StateMachine.add('INTERVENTION', intervention_base_pose(),
					transitions={'retry':'INTERMEDIATE_MOVE', 'no_more_retry':'not_completed','failed':'failed','preempted':'preempted'},
					remapping={'semi_autonomous_mode':'semi_autonomous_mode','intermediate_pose':'intermediate_pose'})
            
            """
			smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
					transitions={'succeeded':'succeeded', 'not_completed':'INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
					remapping={'base_pose':'intermediate_pose'})
            """
            
            smach.StateMachine.add('INTERMEDIATE_MOVE', approach_pose_without_retry(),
                    transitions={'succeeded':'succeeded', 'not_completed':'INTERVENTION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'base_pose':'intermediate_pose'})
            

################################################################################
#Old grasp state machine
#
#grasp assisted by remote operator or KB
################################################################################
class sm_pick_object_asisted(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
                                    input_keys=['target_object_name','semi_autonomous_mode'],
                                    output_keys=['target_object', 'target_object_old_pose', 'grasp_categorisation'])
        
        self.userdata.grasp_categorisation=""
        self.userdata.target_object_old_pose=""
        self.userdata.grasp_configuration = ""
        
        with self:
            smach.StateMachine.add('DETECT_OBJECT-simple', detect_object(max_retries=5),
                    transitions={'succeeded':'SELECT_GRASP', 'retry':'DETECT_OBJECT-simple', 'no_more_retries':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_name':'target_object_name', 'object':'target_object', 'object_pose':'target_object_old_pose' })
            
            
            smach.StateMachine.add('SELECT_GRASP', select_simple_grasp(),
                    transitions={'succeeded':'GRASP_GENERAL', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_categorisation':'grasp_categorisation', 'object':'target_object'})
                
            smach.StateMachine.add('GRASP_GENERAL', simple_grasp(max_retries = 5),
                    transitions={'succeeded':'succeeded', 'retry':'DETECT_OBJECT-simple', 'no_more_retries':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'grasp_categorisation':'grasp_categorisation'})
 
 
################################################################################
#transfer object to tray state machine
#
#
################################################################################
class sm_transfer_object_to_tray(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['grasp_categorisation','surface_distance'])
        
        self.userdata.post_table_pos=""
    
        with self:
            smach.StateMachine.add("SELECT_POST_TABLE_POS", select_post_table_pose(),
                transitions={'succeeded':'MOVE_TO_POST_TABLE_POS','failed':'failed','preempted':'preempted'},
                remapping={'post_table_pos': 'post_table_pos'})               
            smach.StateMachine.add('MOVE_TO_POST_TABLE_POS', approach_pose_without_retry(),
                transitions={'succeeded':'PUT_OBJECT_ON_TRAY','failed':'failed','preempted':'preempted'},
                remapping={'base_pose':'post_table_pos'})                              
            smach.StateMachine.add('PUT_OBJECT_ON_TRAY', put_object_on_tray(),
                transitions={'succeeded':'succeeded', 'failed':'failed','preempted':'preempted', 'not_completed':'not_completed'},
                remapping={'grasp_categorisation':'grasp_categorisation', 'surface_distance':'surface_distance'})       
            
################################################################################
#environment object update state machine
#
#
################################################################################    

class sm_enviroment_model_update(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_base_pose_list'],
                                    output_keys=['reference_to_map']
                                    )
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

            
class sm_enviroment_object_update(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_name_list','scan_pose_list'],
                                    output_keys=['target_object_pose_list']
                                    )
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

            
class sm_enviroment_object_verification_simple(smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                    input_keys=['target_object_hh_id', 'target_object_pose'],
                                    output_keys=['verified_target_object_pose']
                                    )
        self.userdata.angle_range=0.4
        
        with self:    
            smach.StateMachine.add('UPDATE_ENVIROMENT', UpdateEnvMap(),
                    transitions={'succeeded':'VERIFY_OBJECT', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'angle_range':'angle_range'})  
            smach.StateMachine.add('VERIFY_OBJECT', VerifyObject(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'object_id':'target_object_hh_id',
                               'target_object_pose':'target_object_pose',
                               'verified_target_object_pose':'verified_target_object_pose'
                               })           
            

    
            
