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
import roslib; roslib.load_manifest('srs_decision_making')

import rospy
import smach
import smach_ros
import time
import tf
import actionlib
import gc

from std_msgs.msg import String, Bool, Int32
from cob_srvs.srv import Trigger
from geometry_msgs.msg import *

import srs_decision_making.msg as xmsg


"""
This file contains (or import) basic states for SRS high level state machines.

The following states are designed for generic error handling in unstructured environment  

Generic states of interventions for unexpected situation include:

    intervention_base_pose()
    # this can be used to adjust scan position, grasp position or as a intermediate goal for navigation 
    # semi_autonomous_mode == True
    # get a new base pose from UI, 
    # semi_autonomous_mode == False
    # robot has to reason by itself, get a new base pose from KB.
        
    intervention_key_region()
    # get a interested region and then pass it to the object detector
    # semi_autonomous_mode == True
    # get key_region from UI, 
    # semi_autonomous_mode == False
    # robot has to reason by itself, get a new key_region from KB.
    
    intervention_grasp_selection()
    # get a grasp configuration and then pass it to the manipulation module
    # semi_autonomous_mode == True
    # get grasp configuration from UI, 
    # semi_autonomous_mode == False
    # robot has to reason by itself, get a new grasp configuration from KB.    
    
    user_intervention_action_sequence()
    # get an action sequence from UI and then pass it to the  SRS knowledge service to update the existing action sequence

    semantic_dm()
	# This is the heart of the decision making, it monitor and control task execution based on pre-stored knowledge 
	
	
Others
    initialise()
    #initialisation for a given task

"""

"""
Dummy states about navigation, detection and grasp are imported from ipa_examples_mod.py for testing purpose
They should be imported from other SRS components in the future  
"""
from ipa_examples_mod import *



"""
current_task_info is a global shared memory for exchanging task information among statuses 

smach is slow on passing large amount of userdata. Hence they are stored under goal_structure as global variable

srs_dm_action perform one task at a time and maintain a unique session id.  
"""


class goal_structure():   
	
    def __init__(self):
        
        #goal of the high level task
        self.task_name =""
        
        #task parameter
        self.task_parameter=""
        
        #Information about last step, use Last_step_info_msg 
        self.last_step_info = list()
        
        #customised pre-empty signal received or not
        self.preemptied = False
        
        ## backward compatible need to be revised after the integration meeting         
        #feedback publisher, intervention required
        self.pub_fb = rospy.Publisher('fb_executing_solution', Bool)
        #feedback publisher, operational state
        self.pub_fb2 = rospy.Publisher('fb_executing_state', String)
        ## backward compatible need to be revised after the integration meeting  
    
    def reset(self):
        
        self.__init__()
        gc.collect()
        

current_task_info = goal_structure() 


# get a new base pose from UI, this can be used to adjust scan position, grasp position or as a intermediate goal for navigation 
# output is a intermediate pose for navigation or a new scan position
class intervention_base_pose(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','no_more_retry','failed'],
                                input_keys=['semi_autonomous_mode'],
                                output_keys=['intermediate_pose'])
        global current_task_info         
        self.pub_fb = current_task_info.pub_fb
        self.pub_fb2 = current_task_info.pub_fb2
        self.count = 0

    def execute(self,userdata):
        
        if userdata.semi_autonomous_mode == True:
            # user intervention is possible
            while not rospy.is_shutdown():
                
                self.pub_fb.publish(True)
                rospy.sleep(1.0)
                self.count += 1
                self.pub_fb2.publish("need user intervention for base pose")    
                rospy.loginfo("State : need user intervention for base pose")
                current_state= 'need user intervention for base pose'
                exceptional_case_id=1 # need user intervention for base pose
                rospy.wait_for_service('message_errors')                       
                s = xsrv.errorsResponse()
                try:
                    message_errors = rospy.ServiceProxy('message_errors',xsrv.errors)               
                    s = message_errors(current_state, exceptional_case_id)
                    print s.solution
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e           
                    self.pub_fb.publish(False)   
                    return 'failed'
                print (s)             
                
                if  (s.giveup == 1):
                    return 'no_more_retry'
                else:
                    userdata.intermediate_pose = s.solution.__str__()#"home"#[1.0, 3.0, 0.0]"
                    rospy.loginfo("New intermediate target is :%s", s.solution.__str__())    
                    return 'retry'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a new scanning or grasping position
            """
            userdata.intermediate_pose = "kitchen"   
            return 'retry'
            

#get a interested region from UI or KB and then pass it to the object detector
#intervention to highlight key interested region
class intervention_key_region(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','no_more_retry','failed'],
                                input_keys=['semi_autonomous_mode'],
                                output_keys=['key_region'])
        global current_task_info         
        self.pub_fb = current_task_info.pub_fb
        self.pub_fb2 = current_task_info.pub_fb2
        self.count = 0
        
    def execute(self,userdata):
        
        if userdata.semi_autonomous_mode == True:
            # user intervention is possible
            # user specify key region on interface device for detection
            """
            under development IPA+BAS
            UI specify a key region based on video stream or map 
            The key region is translated into the robot coordination to assist current detection
            """
            userdata.key_region = ""   
            return 'failed'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a key region. It is then translated into the robot coordination and assist current detection
            """
            userdata.key_region = ""   
            return 'failed'
            
        

# get a grasp configuration from UI or KB and then pass it to the manipulation module
# user intervention on specific configuration for grasp
class intervention_grasp_selection(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','no_more_retry','failed'],
                                input_keys=['semi_autonomous_mode'],                                
                                output_keys=['grasp_conf'])
        global current_task_info         
        self.pub_fb = current_task_info.pub_fb
        self.pub_fb2 = current_task_info.pub_fb2
        self.count = 0
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
    
        if userdata.semi_autonomous_mode == True:
            # user intervention is possible
            # user specify key region on interface device for detection
            """
            under development ROB+BAS
            UI specify a grasp configuration for the next grasp
            The configuration is passed to grasp 
            """
            userdata.grasp_conf = ""   
            return 'retry'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a grasp conf. It is then pass to the grasp_general 
            """
            userdata.grasp_conf = ""   
            return 'retry'
            
        

# get an action sequence from UI and then pass it to the SRS knowledge service to update the existing action sequence
# 
class user_intervention_action_sequence(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['retry','no_more_retry','failed'],
                                output_keys=['action_sequence'])
        global current_task_info         
        self.pub_fb = current_task_info.pub_fb
        self.pub_fb2 = current_task_info.pub_fb2
        self.count = 0
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        updated action sequence for completing current task
        """
        userdata.action_sequence = ""   
        return 'failed'  

#connection to the srs knowledge_ros_service
class semantic_dm(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','failed','preempted','navigation','detection','simple_grasp','open_door','env_object_update'],
                             input_keys=['target_object_name','target_base_pose','target_object_pose'],
                             output_keys=['target_object_name',
                                          'target_base_pose',
                                          'semi_autonomous_mode',
                                          'target_object_pose'])
        
        
        self.pub_fb = rospy.Publisher('fb_executing_solution', Bool)
        self.pub_fb2 = rospy.Publisher('fb_executing_state', String)
        self.count = 0
        
    def execute(self,userdata):     
        global current_task_info
        
        #call srs ros knowledge service for solution
        
        #dummy code for testing
        userdata.semi_autonomous_mode=False
        
        if current_task_info.task_name=="get" and current_task_info.task_parameter=="milk":
            userdata.target_base_pose="table1"
            userdata.target_object_name="milk_box1"    
                    
        else: 
            userdata.target_base_pose="kitchen"
            userdata.target_object_name="none"
            

        
        rospy.loginfo("self.count: %s", self.count)
        rospy.sleep(1)
        
        if self.count>0:     
            current_task_info.last_step_name = "semantic_dm"

            last_step_info = xmsg.Last_step_info()
            last_step_info.step_name = "semantic_dm"
            last_step_info.outcome = 'succeeded'
            last_step_info.semi_autonomous_mode = False
            
            #recording the information of last step
            current_task_info.last_step_info.append(last_step_info)
            
            return 'succeeded'
        else:

            last_step_info = xmsg.Last_step_info()
            last_step_info.step_name = "semantic_dm"
            last_step_info.outcome = 'random'
            last_step_info.semi_autonomous_mode = False
            self.count+=1
            #recording the information of last step
            current_task_info.last_step_info.append(last_step_info)

            return 'navigation'



#initialisation for a given task
class initialise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        
    def execute(self,userdata):
        
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = "initialise"
        last_step_info.outcome = 'succeeded'
        last_step_info.semi_autonomous_mode = False
        
        #recording the information of last step
        global current_task_info
        current_task_info.last_step_info.append(last_step_info)
        current_task_info.session_id = 123456

        return 'succeeded'
    
    
#verify_object FROM PRO+IPA, the interface still need to be clarified 
class verify_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_verified','no_object_verified','failed'],
                                input_keys=['reference_to_map','object_name'],
                                output_keys=['updated_object'])
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        Extract objects from current point map
        """
        updated_object = ""   #updated pose information about the object
        return 'failed'  

#scan environment from IPA, the interface still need to be clarified    
class update_env_model(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'],
                                output_keys=['reference_to_map'])
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        Get current point map
        """
        map_reference = ""   
        return 'succeeded'      
