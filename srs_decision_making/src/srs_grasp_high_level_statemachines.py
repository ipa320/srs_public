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
#simple grasp from IPA
from simple_grasp_states import *
#grasp based on configuration from ROB

#arm manipulation from IPA
from move_arm_states import *

try:
    #grasp state from ROB, need working openrave
    from srs_grasp_states import *
except ImportError:
    rospy.logwarn ('Package srs_grasping NOT ready! You can only use simple grasp NOT srs grasp')
    # the package is not ready use dummy function instead
    from srs_grasp_dummy_states import *
    
try:
    #assisted arm manipulation from BUT
    from arm_manip_states import *
except ImportError:
    rospy.logwarn ('Package srs_assisted_arm_navigation NOT ready, You can NOT use srs assisted grasp')
    # the package is not working use dummy function instead
    from arm_manip_dummy_states import *


#import service form semantic KB
from srs_knowledge.srv import *

#import other state machines
from srs_common_high_level_statemachines import * 
from srs_detection_high_level_statemachines import *

#from generic_grasp_state import *

# This is come from srs_object_verification, the name should be updated to avoid confusion
#from generic_states import *

"""
This file contains high level state machines for grasp in decision making.
The actual implementation of the states can be found from 
    srs_states packages

Depend on if there is any user client connected and the type of the user client, 
state machines below may switch between semi-autonomous or fully-autonomous mode mode.

If semi_autonomous_mode=True Generic states of user interventions will be loaded for error handling
Otherwise, generic states based on semantic kb for error handling will be loaded 

The default mode is semi_autonomous_mode=False assuming no UI connected to the robot

    sm_simple_grasp()
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

################################################################################
#grasp state machine
#
#grasp simple
################################################################################
class sm_srs_grasp_simple (smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
            input_keys=['target_object_name','target_object_id', 'target_object','target_workspace_name','semi_autonomous_mode'],
            output_keys=['grasp_categorisation', 'surface_distance'])
        self.userdata.surface_distance = -1000 # default value for surface_distance, if the value is negative, the parameter will be ignored. 
              
        self.max_retries = 5  # default value for max retries 
        try:
            self.max_retries = rospy.get_param("srs/common/grasp_max_retries")
        except Exception, e:
            rospy.INFO('can not read parameter of max retries, use the default value')
        
        
        with self:         
            smach.StateMachine.add('SELECT_GRASP-simple', select_simple_grasp(),
                    transitions={'succeeded':'GRASP-simple', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_categorisation':'grasp_categorisation', 'object':'target_object'})
                
            smach.StateMachine.add('GRASP-simple', simple_grasp(self.max_retries),
                    transitions={'succeeded':'succeeded', 'retry':'GRASP-simple', 'no_more_retries':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'grasp_categorisation':'grasp_categorisation'})


################################################################################
#grasp state machine
#
#grasp assisted by remote operator or KB
################################################################################
class sm_srs_grasp_assisted (smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_object_name','target_object_id', 'target_object','target_workspace_name','semi_autonomous_mode'],
            output_keys=['grasp_categorisation', 'surface_distance'])
        
        self.userdata.grasp_configuration = ""   #list of possible grasp configuration
        self.userdata.poses = ""                 #list of pre grasp pose of the above configuration
        self.userdata.base_pose = ""             #best base pose for grasp
        self.userdata.index_of_the_selected_pose = 1   #the id of the pre grasp which has been reached
        self.pose_of_the_target_object = ''
        self.bb_of_the_target_object = ''
        
        self.max_retries = 5  # default value for max retries 
        self.detection_type = 'simple'

        try:
            self.detection_type = rospy.get_param("srs/detection_type")
        except Exception, e:
            rospy.INFO('can not read parameter of detection type, use the default value')

        try:
            self.max_retries = rospy.get_param("srs/common/grasp_max_retries")
        except Exception, e:
            rospy.INFO('can not read parameter of max retries, use the default value')
        
        
        with self:          
                #guided grasp with simple detection      
                smach.StateMachine.add('PREPARE', assisted_arm_navigation_prepare(),
                    transitions={'succeeded':'GRASP_SELECT', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'pose_of_the_target_object':'pose_of_the_target_object', 'bb_of_the_target_object':'bb_of_the_target_object', 'object':'target_object'})
                                
                smach.StateMachine.add('GRASP_SELECT', select_srs_grasp(),
                    transitions={'succeeded':'GRASP_MOVE_ARM', 'not_possible':'GRASP_MOVE_ARM', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_configuration':'grasp_configuration', 'poses':'poses', 'object':'target_object', 'target_object_id':'target_object_id'})
            
                smach.StateMachine.add('GRASP_MOVE_ARM', move_arm_to_given_positions_assisted(),
                    transitions={'succeeded':'GRASP_SRS_GRASP', 'not_completed':'not_completed', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'name_of_the_target_object':'target_object_name', 'list_of_target_positions':'poses', 'pose_of_the_target_object':'pose_of_the_target_object', 'bb_of_the_target_object':'bb_of_the_target_object'})
            
                smach.StateMachine.add('GRASP_SRS_GRASP', srs_grasp(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'grasp_configuration_id':'index_of_the_selected_pose', 'grasp_configuration':'grasp_configuration', 'grasp_categorisation':'grasp_categorisation', 'surface_distance':'surface_distance'})
            

################################################################################
#grasp state machine
#
#grasp planned by planner
################################################################################
class sm_srs_grasp_planned (smach.StateMachine):
    def __init__(self):    
        smach.StateMachine.__init__(self, 
            outcomes=['succeeded', 'not_completed', 'failed', 'stopped', 'preempted'],
            input_keys=['target_object_name','target_object_id', 'target_object','target_workspace_name','semi_autonomous_mode'],
            output_keys=['grasp_categorisation','surface_distance'])
        
        self.userdata.grasp_configuration = ""   #list of possible grasp configuration
        self.userdata.poses = ""                 #list of pre grasp pose of the above configuration
        self.userdata.base_pose = ""             #best base pose for grasp
        self.userdata.index_of_the_selected_pose = 0   #the id of the pre grasp which has been reached, the default value is the first one
        self.userdata.grasp_categorisation = ""
        self.max_retries = 5  # default value for max retries 
        self.detection_type = 'simple'
                   
        try:
            self.detection_type = rospy.get_param("srs/detection_type")
        except Exception, e:
            rospy.INFO('can not read parameter of detection type, use the default value')

        try:
            self.max_retries = rospy.get_param("srs/common/grasp_max_retries")
        except Exception, e:
            rospy.INFO('can not read parameter of max retries, use the default value')
            
        
        step_after_grasp_select = 'GRASP_SRS_GRASP'
        
        self.ipa_arm_navigation = 'False'
        
        #default value is failed, would be replaced by proper value below:
        self.step_after_grasp_select = 'failed'
        
        try:
            self.ipa_arm_navigation = rospy.get_param("srs/ipa_arm_navigation")
        except Exception, e:
            rospy.INFO('can not read parameter of srs/ipa_arm_navigation, use the default value planned arm navigation disabled')
        
        if self.ipa_arm_navigation.lower() == 'true':
            #move arm planned before grasp
            step_after_grasp_select = 'GRASP_MOVE_ARM'
        else:
            #move arm unplanned before grasp
            step_after_grasp_select = 'GRASP_SRS_GRASP'
        
        
        if self.detection_type.lower() == 'simple':
            with self:          
                #guided grasp with simple detection        
                smach.StateMachine.add('GRASP_SELECT', select_srs_grasp(),
                    transitions={'succeeded':step_after_grasp_select, 'not_possible':'GRASP_BEST_BASE_POSE_ESTIMATION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_configuration':'grasp_configuration', 'poses':'poses', 'object':'target_object', 'target_object_id':'target_object_id'})
            
                smach.StateMachine.add('GRASP_MOVE_ARM', move_arm_planned(),
                    transitions={'succeeded':'GRASP_SRS_GRASP', 'not_completed':'GRASP_SRS_GRASP', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'poses':'poses', 'index_of_the_selected_pose':'pose_id'})
            
                smach.StateMachine.add('GRASP_SRS_GRASP', srs_grasp(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted', 'retry':'GRASP_BEST_BASE_POSE_ESTIMATION'},
                    remapping={'grasp_configuration_id':'index_of_the_selected_pose', 'grasp_configuration':'grasp_configuration', 'grasp_categorisation':'grasp_categorisation', 'surface_distance':'surface_distance' })
            
                smach.StateMachine.add('GRASP_BEST_BASE_POSE_ESTIMATION', grasp_base_pose_estimation(),
                    transitions={'retry':'GRASP_MOVE_BASE', 'not_retry':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'target_workspace_name':'target_workspace_name', 'base_pose':'base_pose'})
                
                smach.StateMachine.add('GRASP_MOVE_BASE', approach_pose_without_retry(),
                    transitions={'succeeded':'GRASP_DETECT_SIMPLE', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'grasp_configuration':'grasp_configuration', 'base_pose':'base_pose'})
                
                smach.StateMachine.add('GRASP_DETECT_SIMPLE', sm_simple_detection(),
                    transitions={'succeeded':'GRASP_SELECT', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'target_object':'target_object', 'target_object_name':'target_object_name', 'target_object_id':'target_object_id', 'target_workspace_name':'target_workspace_name'})

        else:
            with self: 
                # guided grasp with assisted detection
                smach.StateMachine.add('GRASP_SELECT', select_srs_grasp(),
                    transitions={'succeeded':step_after_grasp_select, 'not_possible':'GRASP_BEST_BASE_POSE_ESTIMATION', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'grasp_configuration':'grasp_configuration', 'poses':'poses', 'object':'target_object', 'target_object_id':'target_object_id'})
            
                smach.StateMachine.add('GRASP_MOVE_ARM', move_arm_planned(),
                    transitions={'succeeded':'GRASP_SRS_GRASP', 'not_completed':'GRASP_SRS_GRASP', 'failed':'failed', 'preempted':'preempted'},
                    remapping={'poses':'poses', 'index_of_the_selected_pose':'pose_id'})
            
                smach.StateMachine.add('GRASP_SRS_GRASP', srs_grasp(),
                    transitions={'succeeded':'succeeded', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted', 'retry':'GRASP_BEST_BASE_POSE_ESTIMATION'},
                    remapping={'grasp_configuration_id':'index_of_the_selected_pose', 'grasp_configuration':'grasp_configuration', 'grasp_categorisation':'grasp_categorisation', 'surface_distance':'surface_distance' })
            
                smach.StateMachine.add('GRASP_BEST_BASE_POSE_ESTIMATION', grasp_base_pose_estimation(),
                    transitions={'retry':'GRASP_MOVE_BASE', 'not_retry':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'target_workspace_name':'target_workspace_name', 'base_pose':'base_pose'})
                
                smach.StateMachine.add('GRASP_MOVE_BASE', approach_pose_without_retry(),
                    transitions={'succeeded':'GRASP_DETECT_ASSISTED', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'object':'target_object', 'grasp_configuration':'grasp_configuration', 'base_pose':'base_pose'})
                
                smach.StateMachine.add('GRASP_DETECT_ASSISTED', sm_assisted_detection(),
                    transitions={'succeeded':'GRASP_SELECT', 'not_completed':'not_completed', 'failed':'failed','preempted':'preempted'},
                    remapping={'target_object':'target_object', 'target_object_name':'target_object_name', 'target_object_id':'target_object_id', 'target_workspace_name':'target_workspace_name'})
            


