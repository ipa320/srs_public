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
import roslib; roslib.load_manifest('srs_decision_making_experimental')
#roslib.load_manifest('knowledge_ros_service')

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

import srs_decision_making_experimental.msg as xmsg
import srs_decision_making_experimental.srv as xsrv
from srs_knowledge.srv import *
from srs_knowledge.msg import *


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
        
        #reference to the action server
        self._as=""
        
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
            while not (rospy.is_shutdown() or self.preempt_requested()) :
                                
                global current_task_info
                _feedback=xmsg.ExecutionFeedback()
                _feedback.current_state = "need user intervention for base pose"
                _feedback.solution_required = True
                _feedback.exceptional_case_id = 1
                current_task_info._as.publish_feedback(_feedback)
                rospy.sleep(1)
                
                self.count += 1 
                rospy.loginfo("State : need user intervention for base pose")
                rospy.wait_for_service('message_errors')                       
                s = xsrv.errorsResponse()
                
                current_state= 'need user intervention for base pose'
                exceptional_case_id=1 # need user intervention for base pose
                
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
            return 'failed'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a new scanning or grasping position
            """
            userdata.intermediate_pose = "kitchen"   
            return 'no_more_retry'
            

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
            return 'no_more_retry'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a key region. It is then translated into the robot coordination and assist current detection
            """
            userdata.key_region = ""   
            return 'no_more_retry'
            
        

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
            userdata.grasp_conf = "Top"   
            return 'retry'
        else:
            # no user intervention, UI is not connected or not able to handle current situation 
            # fully autonomous mode for the current statemachine, robot try to handle error by it self with semantic KB
            """
            call srs knowledge ros service for a grasp conf. It is then pass to the grasp_general 
            """
            userdata.grasp_conf = "Top"   
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
        return 'no_more_retry'  

#connection to the srs knowledge_ros_service
class semantic_dm(smach.State):

    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded','failed','preempted','navigation','detection','simple_grasp','env_object_update','deliver_object','charging'],
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
        userdata.semi_autonomous_mode=True

        ##############################################
        # get Next Action From Knowledge_ros_service
        ##############################################
        print 'Plan next Action service'
        rospy.wait_for_service('plan_next_action')
        try:
            next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)

            # decide result ()
            # current_task_info.last_step_info.append(last_step_info)
            # current_task_info.session_id = 123456

            result = (1, 1, 1)
            # print current_task_info.last_step_info
            print '+++++++++++ action acquired ++++++++++++++'
            print current_task_info.last_step_info;
            print '+++++++++++ Last Step Info LEN+++++++++++++++'
            print len(current_task_info.last_step_info)
            len_step_info = len(current_task_info.last_step_info)
            
        
            if not current_task_info.last_step_info:
                print 'first action acquired ++++ '
                result = (0, 0, 0)   ## first action. does not matter this. just to keep it filled
            elif current_task_info.last_step_info and current_task_info.last_step_info[len_step_info - 1].outcome == 'succeeded':
                # convert to the format that knowledge_ros_service understands
                result = (0, 0, 0)
            elif current_task_info.last_step_info[len_step_info - 1].outcome == 'not_completed':
                print 'Result return failed'
                if current_task_info.last_step_info[len_step_info - 1].step_name == 'sm_approach_pose_assisted':
                    result = (1, 0, 0)
                elif current_task_info.last_step_info[len_step_info - 1].step_name == 'sm_detect_asisted_pose_region':
                    result = (1, 1, 0)
                elif current_task_info.last_step_info[len_step_info - 1].step_name == 'sm_pick_object_asisted':
                    result = (0, 1, 1)
                else:
                    result = (1, 1, 1)


            print result
            print '########## Result ###########'

            

            resp1 = next_action(current_task_info.session_id, result, None)
            if resp1.nextAction.status == 1:
                print 'succeeded'
                return 'succeeded'
            elif resp1.nextAction.status == -1:
                print 'failed'
                return 'failed'
            
            print resp1.nextAction
            # else should be 0: then continue executing the following

            if resp1.nextAction.actionType == 'generic':
                if resp1.nextAction.generic.actionInfo[0] == 'charging':
                    nextStep = 'charging'
                    userdata.target_base_pose = [float(resp1.nextAction.generic.actionInfo[1]), float(resp1.nextAction.generic.actionInfo[2]), float(resp1.nextAction.generic.actionInfo[3])]                    
                    return nextStep
                elif resp1.nextAction.generic.actionInfo[0] == 'move':
                    nextStep = 'navigation'
                    userdata.target_base_pose = [float(resp1.nextAction.generic.actionInfo[1]), float(resp1.nextAction.generic.actionInfo[2]), float(resp1.nextAction.generic.actionInfo[3])]                    
                    return nextStep
                elif resp1.nextAction.generic.actionInfo[0] == 'deliver_object':
                    nextStep = 'deliver_object'
                    userdata.target_base_pose = [float(resp1.nextAction.generic.actionInfo[1]), float(resp1.nextAction.generic.actionInfo[2]), float(resp1.nextAction.generic.actionInfo[3])]                    
                    return nextStep
                elif resp1.nextAction.generic.actionInfo[0] == 'detect':
                                      
                    nextStep = 'detection'
                    #TODO should confirm later if name or id used !!!!!!!!
		    ####  HARD CODED FOR TESTING ##

                    userdata.target_object_name = 'milk_box'
                    return nextStep
		    ####  END OF HARD CODED FOR TESTING ##

                    #userdata.target_object_name = resp1.nextAction.pa.aboxObject.object_id
                    # should be updated to object_id in future


                
            else:
                print 'No valid actionFlags'
                print resp1.nextAction.actionFlags
                #nextStep = 'No_corresponding_action???'
                nextStep = 'failed'
                return 'failed'




                """
            if resp1.nextAction.actionFlags == (0, 1, 1):
                if resp1.nextAction.ma.ifWaitObjectTaken:
			nextStep = 'deliver_object'
		else:
	                nextStep = 'navigation'
                #userdata.target_base_pose = resp1.nextAction.ma.targetPose2D
                userdata.target_base_pose = [resp1.nextAction.ma.targetPose2D.x, resp1.nextAction.ma.targetPose2D.y, resp1.nextAction.ma.targetPose2D.theta]

            elif resp1.nextAction.actionFlags == (0, 0, 1):
                
                nextStep = 'detection'
                userdata.target_base_pose = [resp1.nextAction.ma.targetPose2D.x, resp1.nextAction.ma.targetPose2D.y, resp1.nextAction.ma.targetPose2D.theta]

                #TODO should confirm later if name or id used !!!!!!!!
		####  HARD CODED FOR TESTING ##

		if resp1.nextAction.pa.aboxObject.object_id == 1:
			userdata.target_object_name = 'milk_box'
 		else:
			userdata.target_object_name = 'milk_box'

		####  END OF HARD CODED FOR TESTING ##

                #userdata.target_object_name = resp1.nextAction.pa.aboxObject.object_id
                # should be updated to object_id in future
            elif resp1.nextAction.actionFlags == (1, 0, 0):
                nextStep = 'simple_grasp'
                #userdata.target_object_name = resp1.nextAction.pa.aboxObject.name

		####  HARD CODED FOR TESTING ##

		if resp1.nextAction.pa.aboxObject.object_id == 1:
			userdata.target_object_name = 'milk_box'
 		else:
			userdata.target_object_name = 'milk_box'

		####  END OF HARD CODED FOR TESTING ##

		#userdata.target_object_name = resp1.nextAction.pa.aboxObject.object_id
		# should be updated to object_id in future            
            else:
                print 'No valid actionFlags'
                print resp1.nextAction.actionFlags
                nextStep = 'No_corresponding_action???'
                return 'failed'
            #return resp1.nextAction
                """
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e        
            return 'failed'

        return nextStep

        ##############################################        
        ### End of GetNextAction######################
        ##############################################

        
        """
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
        """


#initialisation for a given task
class initialise(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        #self.count=0

        
    def execute(self,userdata):
               
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = "initialise"
        last_step_info.outcome = 'succeeded'
        last_step_info.semi_autonomous_mode = False
        
        #recording the information of last step
        global current_task_info
        current_task_info.last_step_info.append(last_step_info)
                
        #current_task_info.session_id = 123456
        

        return 'succeeded'
    
#prepare the robot for the task
class prepare_robot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        #self.count=0

        
    def execute(self,userdata):
               
        last_step_info = xmsg.Last_step_info()
        last_step_info.step_name = "prepare_robot"
        last_step_info.outcome = 'succeeded'
        last_step_info.semi_autonomous_mode = False
        
        #recording the information of last step
        global current_task_info
        current_task_info.last_step_info.append(last_step_info)
                
        #current_task_info.session_id = 123456
        
        #initialisation of the robot
        # move to initial positions
        global sss
        handle_torso = sss.move("torso", "home", False)
        handle_tray = sss.move("tray", "down", False)
        handle_arm = sss.move("arm", "folded", False)
        handle_sdh = sss.move("sdh", "cylclosed", False)
        handle_head = sss.move("head", "front", False)
    
        
        # wait for initial movements to finish
        handle_torso.wait()
        handle_tray.wait()
        handle_arm.wait()
        handle_sdh.wait()
        handle_head.wait()
        
        

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
