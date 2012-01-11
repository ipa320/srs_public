# ROS imports
import roslib
roslib.load_manifest('srs_decision_making_experimental')
import copy
import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Int32
from cob_srvs.srv import Trigger
from math import *
import time
import tf
from kinematics_msgs.srv import *

import actionlib

# include script server, to move the robot
from simple_script_server import *
sss = simple_script_server()

# msg imports
from geometry_msgs.msg import *

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *
from gazebo.srv import *
import gazebo.msg as gazebo
from cob_tray_sensors import *
#import geometry_msgs.msg as geomery
#from gazebo.srv import SetModelState
from cob_mmcontroller.msg import *
from kinematics_msgs.srv import *




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
        self._srs_as=""
        
        ## backward compatible need to be revised after the integration meeting         
        #feedback publisher, intervention required
        self.pub_fb = rospy.Publisher('fb_executing_solution', Bool)
        #feedback publisher, operational state
        self.pub_fb2 = rospy.Publisher('fb_executing_state', String)
        ## backward compatible need to be revised after the integration meeting  
        
        self.object_in_hand = False
        
        # this can be replaced by the tray service with real robot
        self.object_on_tray = False
        
        self.arm_folded_ready_for_transfer = False    #arm in hold or folded position
    
    #checking if the current atomic operation can be stopped or not
    #true can be stopped in the middle
    def stopable(self):
        #List of conditions which robot should not be stopped in the middle
        
        #condition 1:
        #If a object has been grasped, operation should not be stopped until the object is released or arm is folded ready for transfer
        if self.object_in_hand and not self.arm_folded_ready_for_transfer:
            return False
        #The condition can be expanded 
        
        #in all other cases, atomic action can be stopped in the middle
        else:
            return True
    
    def reset(self):
        
        self.__init__()
        gc.collect()
        

current_task_info = goal_structure() 



            

"""
Below dummy generic states are copied and modified based on IPA examples for testing purpose
They should be replaced by real states from other SRS components in the future  

Basic states related to robot includes:

approach_pose()
approach_pose_without_retry()
select_grasp()
grasp_side()
grasp_top()
open_door()
put_object_on_tray()
detect_object()

Only dummy outputs are given for testing purpose
"""


## Approach pose state (without retry)
#
# This state tries once to move the robot to the given pose.
class approach_pose_without_retry(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_completed','failed', 'preempted'],
            input_keys=['base_pose'])

        self.pose = pose
        self.counter =0
        self.mode = "linear"
        #self.mode = "omni"

    def execute(self, userdata):
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # determine target position
        if self.pose != "":
            pose = self.pose
        elif type(userdata.base_pose) is str:
            pose = userdata.base_pose
        elif type(userdata.base_pose) is list:
            pose = []
            pose.append(userdata.base_pose[0])
            pose.append(userdata.base_pose[1])
            pose.append(userdata.base_pose[2])
        else: # this should never happen
            rospy.logerr("Invalid userdata 'pose'")
            return 'failed'

        # try reaching pose
        handle_base = sss.move("base", pose, False, self.mode)
        move_second = False
        


        timeout = 0
        while not self.preempt_requested():
            try:
                #print "base_state = ", handle_base.get_state()
                if (handle_base.get_state() == 3) and (not move_second):
                    # do a second movement to place the robot more exactly
                    handle_base = sss.move("base", pose, False, self.mode)
                    move_second = True
                elif (handle_base.get_state() == 3) and (move_second):
                    return 'succeeded'        
                elif (handle_base.get_state() == 2 or handle_base.get_state() == 4):  #error or paused
                    rospy.logerr("base not arrived on target yet")
                    return 'not_completed'
            except rospy.ROSException, e:
                error_message = "%s"%e
                rospy.logerr("unable to check hdl_base state, error: %s", error_message)
                rospy.sleep(0.5)

            # check if service is available
            service_full_name = '/base_controller/is_moving'
            try:
                rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
            except rospy.ROSException, e:
                error_message = "%s"%e
                rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
                return 'failed'
        
            # check if service is callable
            try:
                is_moving = rospy.ServiceProxy(service_full_name,Trigger)
                resp = is_moving()
            except rospy.ServiceException, e:
                error_message = "%s"%e
                rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                return 'failed'
        
            # evaluate service response
            if not resp.success.data: # robot stands still
                if timeout > 10:
                    sss.say(["I can not reach my target position because my path or target is blocked, I will abort."],False)
		
                    try:
                        rospy.wait_for_service('base_controller/stop',10)
                        stop = rospy.ServiceProxy('base_controller/stop',Trigger)
                        resp = stop()
                    except rospy.ServiceException, e:
                        error_message = "%s"%e
                        rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                    except rospy.ROSException, e:
                        error_message = "%s"%e
                        rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)		
	            return 'not_completed'
                else:
                    timeout = timeout + 1
                    rospy.sleep(1)
            else:
                timeout = 0
        if self.preempt_requested():
            self.service_preempt()
            #handle_base.set_failed(4)
            handle_base.client.cancel_goal()
            return 'preempted'
        return 'failed'




## Detect state
#
# This state will try to detect an object.
class detect_object(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','retry','no_more_retries','failed','preempted'],
            input_keys=['object_name','key_region'],
            output_keys=['object','object_pose'])

        self.object_list = DetectionArray()
        self.max_retries = max_retries
        self.retries = 0
        self.object_name = object_name
        self.srv_name_object_detection = '/object_detection/detect_object'
        
        self.torso_poses = []
        self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left_extreme")
        self.listener = tf.TransformListener()

    def execute(self, userdata):
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # checking extra information
        if userdata.key_region == '':
            pass #normal detection
        else:
            pass #detection in bounding box specified by key_region
        
        #add initial value to output keys
        userdata.object = ""
        userdata.object_pose=""
        
        # determine object name
        if self.object_name != "":
            object_name = self.object_name
        elif type(userdata.object_name) is str:
            object_name = userdata.object_name
        else: # this should never happen
            rospy.logerr("Invalid userdata 'object_name'")
            self.retries = 0
            return 'failed'
    
        # check if maximum retries reached
        if self.retries > self.max_retries:
            self.retries = 0
            return 'no_more_retries'
        
        # move sdh as feedback
        sss.move("sdh","cylclosed",False)
        
        # make the robot ready to inspect the scene
        if self.retries == 0: # only move arm, sdh and head for the first try
            sss.say(["I will now search for the " + object_name + "."],False)
            handle_arm = sss.move("arm","folded-to-look_at_table",False)
            handle_torso = sss.move("torso","shake",False)
            handle_head = sss.move("head","back",False)
            
            if self.preempt_requested():
                self.service_preempt()
                #handle_base.set_failed(4)
                handle_arm.client.cancel_goal()
                handle_torso.client.cancel_goal()
                handle_head.client.cancel_goal()
                return 'preempted'
            else:
                handle_arm.wait()
                handle_head.wait()
                handle_torso.wait()
                
        handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) # have an other viewing point for each retry
        
        # move sdh as feedback
        sss.move("sdh","home",False)
        
        # wait for image to become stable
        sss.sleep(2)
        
    
        # check if object detection service is available
        try:
            rospy.wait_for_service(self.srv_name_object_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            self.retries = 0 # no object found within min_dist start value
            return 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
            req = DetectObjectsRequest()
            req.object_name.data = object_name
            res = detector_service(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.retries = 0
            return 'failed'
            
        # check for no objects
        if len(res.object_list.detections) <= 0:
            rospy.logerr("No objects found")
            self.retries += 1
            return 'retry'
        
        # select nearest object in x-y-plane in head_camera_left_link
        min_dist = 2 # start value in m
        obj = Detection()
        for item in res.object_list.detections:
            dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
            if dist < min_dist:
                min_dist = dist
                obj = copy.deepcopy(item)
        
        # check if an object could be found within the min_dist start value
        if obj.label == "":
            rospy.logerr("Object not within target range")
            self.retries += 1
            return 'retry'

        #check if label of object fits to requested object_name
        if obj.label != object_name:
            rospy.logerr("The object name doesn't fit.")
            self.retries += 1
            return 'retry'

        # we succeeded to detect an object
        userdata.object = obj
        object_pose_map = PoseStamped()
        self.retries = 0
        
        try:
            #transform object_pose into base_link
            object_pose_in = obj.pose
            object_pose_in.header.stamp = self.listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
            object_pose_map = self.listener.transformPose("/map", object_pose_in)
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'
        
        print object_pose_map
        
        userdata.object_pose=object_pose_map
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        return 'succeeded'




## Select grasp state
#
# This state select a grasping strategy. A high object will be grasped from the side, a low one from top.
class select_grasp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['object'],
            output_keys=['grasp_categorisation'])
        
        """
        Very simple grasp selection
        This need to be transfered into symbolic grounding service
        """
        self.height_switch = 0.5 # Switch to select top or side grasp using the height of the object over the ground in [m].
        
        self.listener = tf.TransformListener()
        
        #default grasp categorisation
        self.grasp_categorisation = 'side'

    def execute(self, userdata):
        
        try:
            # transform object_pose into base_link
            object_pose_in = userdata.object.pose
            print object_pose_in
            object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
            object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'
        
        if object_pose_bl.pose.position.z >= self.height_switch: #TODO how to select grasps for objects within a cabinet or shelf?
            userdata.grasp_categorisation='side'
        else: 
            userdata.grasp_categorisation= 'top'
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        return 'succeeded'


## Grasp general state
#
# This state will grasp an object with a side grasp
class grasp_general(smach.State):

    def __init__(self, max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'retry', 'no_more_retries', 'failed', 'preempted'],
            input_keys=['object','grasp_categorisation'])
        
        self.max_retries = max_retries
        self.retries = 0
        self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
        self.listener = tf.TransformListener()
        self.stiffness = rospy.ServiceProxy('/arm_controller/set_joint_stiffness', SetJointStiffness)

    def callIKSolver(self, current_pose, goal_pose):
        req = GetPositionIKRequest()
        req.ik_request.ik_link_name = "sdh_grasp_link"
        req.ik_request.ik_seed_state.joint_state.position = current_pose
        req.ik_request.pose_stamped = goal_pose
        resp = self.iks(req)
        result = []
        for o in resp.solution.joint_state.position:
            result.append(o)
        return (result, resp.error_code)

    def execute(self, userdata):
        

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # check if maximum retries reached
        if self.retries > self.max_retries:
            self.retries = 0
            return 'no_more_retries'
        
        # transform object_pose into base_link
        object_pose_in = userdata.object.pose
        object_pose_in.header.stamp = self.listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
        object_pose_bl = self.listener.transformPose("/base_link", object_pose_in)
        
        
        if userdata.grasp_categorisation == 'side':
            # make arm soft TODO: handle stiffness for schunk arm
            try:
                self.stiffness([300,300,300,100,100,100,100])
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                self.retries = 0
                return 'failed'
        
            [new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(-1.552, -0.042, 2.481) # rpy 
            object_pose_bl.pose.orientation.x = new_x
            object_pose_bl.pose.orientation.y = new_y
            object_pose_bl.pose.orientation.z = new_z
            object_pose_bl.pose.orientation.w = new_w
    
            # FIXME: this is calibration between camera and hand and should be removed from scripting level
            object_pose_bl.pose.position.x = object_pose_bl.pose.position.x #- 0.06 #- 0.08
            object_pose_bl.pose.position.y = object_pose_bl.pose.position.y #- 0.05
            object_pose_bl.pose.position.z = object_pose_bl.pose.position.z  #- 0.1
            
            # calculate pre and post grasp positions
            pre_grasp_bl = PoseStamped()
            post_grasp_bl = PoseStamped()
            pre_grasp_bl = copy.deepcopy(object_pose_bl)
            post_grasp_bl = copy.deepcopy(object_pose_bl)
    
            #pre_grasp_bl.pose.position.x = pre_grasp_bl.pose.position.x + 0.10 # x offset for pre grasp position
            #pre_grasp_bl.pose.position.y = pre_grasp_bl.pose.position.y + 0.10 # y offset for pre grasp position
            #post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
            #post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15 # z offset for post grasp position
    
            pre_grasp_bl.pose.position.x = pre_grasp_bl.pose.position.x + 0.10 # x offset for pre grasp position
            pre_grasp_bl.pose.position.y = pre_grasp_bl.pose.position.y + 0.10 # y offset for pre grasp position
            pre_grasp_bl.pose.position.z = pre_grasp_bl.pose.position.z + 0.15 # y offset for pre grasp position
            post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
            post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.17 # z offset for post grasp position
            
        elif userdata.grasp_categorisation == 'top':
            try:
                self.stiffness([100,100,100,100,100,100,100])
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                self.retries = 0
                return 'failed'
        
            # use a predefined (fixed) orientation for object_pose_bl
            [new_x, new_y, new_z, new_w] = tf.transformations.quaternion_from_euler(3.121, 0.077, -2.662) # rpy 
            object_pose_bl.pose.orientation.x = new_x
            object_pose_bl.pose.orientation.y = new_y
            object_pose_bl.pose.orientation.z = new_z
            object_pose_bl.pose.orientation.w = new_w
    
            # FIXME: this is calibration between camera and hand and should be removed from scripting level
            object_pose_bl.pose.position.x = object_pose_bl.pose.position.x #-0.04 #- 0.08
            object_pose_bl.pose.position.y = object_pose_bl.pose.position.y# + 0.02
            object_pose_bl.pose.position.z = object_pose_bl.pose.position.z #+ 0.07
    
            # calculate pre and post grasp positions
            pre_grasp_bl = PoseStamped()
            post_grasp_bl = PoseStamped()
            pre_grasp_bl = copy.deepcopy(object_pose_bl)
            post_grasp_bl = copy.deepcopy(object_pose_bl)
        
            pre_grasp_bl.pose.position.z = pre_grasp_bl.pose.position.z + 0.18 # z offset for pre grasp position
            post_grasp_bl.pose.position.x = post_grasp_bl.pose.position.x + 0.05 # x offset for post grasp position
            post_grasp_bl.pose.position.z = post_grasp_bl.pose.position.z + 0.15 # z offset for post grasp position
        else:
            return 'failed'
            #unknown categorisation
           
            

        # calculate ik solutions for pre grasp configuration
        arm_pre_grasp = rospy.get_param("/script_server/arm/pregrasp_top")
        (pre_grasp_conf, error_code) = self.callIKSolver(arm_pre_grasp[0], pre_grasp_bl)        
        if(error_code.val != error_code.SUCCESS):
            rospy.logerr("Ik pre_grasp Failed")
            self.retries += 1
            return 'retry'
        
        # calculate ik solutions for grasp configuration
        (grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, object_pose_bl)
        if(error_code.val != error_code.SUCCESS):
            rospy.logerr("Ik grasp Failed")
            self.retries += 1
            return 'retry'
        
        # calculate ik solutions for pre grasp configuration
        (post_grasp_conf, error_code) = self.callIKSolver(grasp_conf, post_grasp_bl)
        if(error_code.val != error_code.SUCCESS):
            rospy.logerr("Ik post_grasp Failed")
            self.retries += 1
            return 'retry'    
    
        # execute grasp
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        sss.say(["I am grasping the " + userdata.object.label + " now."],False)
        sss.move("torso","home")
        handle_arm = sss.move("arm", [pre_grasp_conf , grasp_conf],False)
        sss.move("sdh", "cylopen")
        
        if self.preempt_requested():
            self.service_preempt()
            handle_arm.client.cancel_goal()
            return 'preempted'
        else:
            handle_arm.wait()
            
            
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            if userdata.grasp_categorisation == 'side':
                sss.move("sdh", "cylclosed")
            elif userdata.grasp_categorisation == 'side':
                sss.move("sdh", "spherclosed")
            else:
                return 'failed'
                #unknown categorisation
                
            #object is already in hand    
            global current_task_info
            current_task_info.object_in_hand = True
            
            sss.move("arm", [post_grasp_conf, "hold"])
            self.retries = 0     
            return 'succeeded'

class select_post_table_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed','preempted'], output_keys=['post_table_pos'])
    def execute(self, userdata):
        userdata.post_table_pos='home'
        return 'succeeded'
        

class select_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['got_to_next_pose', 'no_more_pose', 'failed','preempted'], 
                             input_keys=['target_base_pose_list'], output_keys=['current_target_base_pose'])
    def execute(self, userdata):
        userdata.current_target_base_pose='home'
        return 'got_to_next_pose'


## Put object on tray side state
#
# This state puts a side grasped object on the tray
class put_object_on_tray(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed' ,'preempted'],
            input_keys=['grasp_categorisation'])
        
    def execute(self, userdata):
        #TODO select position on tray depending on how many objects are on the tray already
        
        # move object to frontside
        handle_arm = sss.move("arm","grasp-to-tray",False)
        sss.sleep(2)
        sss.move("tray","up")
        handle_arm.wait()
        
        # release object
        if userdata.grasp_categorisation == 'side':
            sss.move("sdh","cylopen")
        elif userdata.grasp_categorisation == 'top':
            sss.move("sdh","spheropen")
            
        #object is transfered from hand to tray 
        global current_task_info
        current_task_info.object_in_hand = False
        current_task_info.object_on_tray = True
        

        # move arm to backside again
        handle_arm = sss.move("arm","tray-to-folded",False)
        sss.sleep(3)
        sss.move("sdh","home")
        handle_arm.wait()
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


#verify_object FROM PRO+IPA, the interface still need to be clarified 
class verify_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_verified','no_object_verified','failed','preempted'],
                                input_keys=['reference_to_map','object_name_list'],
                                output_keys=['updated_object_list'])
        
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
        smach.State.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'preempted'],
                                output_keys=['reference_to_map'])
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        Get current point map
        """
        map_reference = ""   
        return 'succeeded'      


    
"""
## Detect state
#
# This state will try to detect an object.
class detect_object(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','retry','no_more_retries','failed'],
            input_keys=['object_name'],
            output_keys=['object'])

        self.object_name = object_name
        

    def execute(self, userdata):
        # determine object name
        if self.object_name != "":
            object_name = self.object_name
        elif type(userdata.object_name) is str:
            object_name = userdata.object_name
        else: # this should never happen
            rospy.logerr("Invalid userdata 'object_name'")
            self.retries = 0
            return 'failed'

        return 'succeeded'





class move_head(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['torso_pose'])
        
        self.torso_poses = []
        self.torso_poses.append("home")
        self.torso_poses.append("left")
        self.torso_poses.append("right")


    def execute(self, userdata):
        sss.move("torso",userdata.torso_pose)
        return 'succeeded'




## Deliver object state
#
# This state will deliver an object which should be on the tray.
class deliver_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'retry', 'failed'])

    def execute(self, userdata):
        
        #sss.say(["Here is your " + userdata.object_name + ". Please help yourself."],False)
        sss.move("torso","nod",False)
        
        try:
            rospy.wait_for_service('/tray/check_occupied',10)
        except rospy.ROSException, e:
            rospy.loginfo("\n\nService not available: %s", e)
	    rospy.loginfo('\n\nIf the tray has been emptied? Please enter Yes/No - Y/N')
	    inp = raw_input()
	    if inp == 'y' or inp == 'Y':
	        
		sss.move("tray","down",False)
                sss.move("torso","nod",False)
		return  'succeeded'
	    else:
		return 'failed'

        time = rospy.Time.now().secs
        loop_rate = rospy.Rate(5) #hz
        while True:
            if rospy.Time.now().secs-time > 20:
                return 'retry'
            try:
                tray_service = rospy.ServiceProxy('/tray/check_occupied', CheckOccupied)            
                req = CheckOccupiedRequest()
                res = tray_service(req)
                print "waiting for tray to be not occupied any more"
                if(res.occupied.data == False):
                    break
            except rospy.ServiceException, e:
                print "Service call failed: %s", e
                return 'failed'
            sss.sleep(2)
        
        sss.move("tray","down",False)
        sss.move("torso","nod",False)
        
        return 'succeeded'
"""

