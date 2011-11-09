# ROS imports
import roslib; roslib.load_manifest('srs_decision_making')

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Int32
from cob_srvs.srv import Trigger

import time
import tf
from kinematics_msgs.srv import *

import actionlib

# include script server, to move the robot
from simple_script_server import simple_script_server
sss = simple_script_server()

# msg imports
from geometry_msgs.msg import *

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *
from gazebo.srv import *


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

## Approach pose state
#
# This state will try forever to move the robot to the given pose.
class approach_pose(smach.State):

    def __init__(self, pose = "", mode = "omni", move_second = "False"):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['base_pose'])

        self.pose = pose
        self.mode = mode
        self.move_second = move_second

    def execute(self, userdata):
        rospy.loginfo('base_pose: %s', userdata.base_pose)
        return 'succeeded'

## Approach pose state (without retry)
#
# This state tries once to move the robot to the given pose.
class approach_pose_without_retry(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['base_pose'])

        self.pose = pose
        self.counter =0
        self.mode = "linear"

    def execute(self, userdata):
        """
        rospy.loginfo("target base pose: %s", userdata.base_pose)
        self.counter=self.counter+1
        rospy.sleep(1)
        if self.counter>1:
            return 'succeeded'
        else:
            return 'failed'
        """
        
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
        while True:
            try:
                print "base_state = ", handle_base.get_state()
                if (handle_base.get_state() == 3) and (not move_second):
                    # do a second movement to place the robot more exactly
                    handle_base = sss.move("base", pose, False, self.mode)
                    move_second = True
                elif (handle_base.get_state() == 3) and (move_second):
                    return 'succeeded'        
                elif (handle_base.get_state() == 2 or handle_base.get_state() == 4):  #error or paused
                    rospy.logerr("base not arrived on target yet")
                    return 'failed'
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
        
            # evaluate sevice response
            if not resp.success.data: # robot stands still
                if timeout > 10:
                    sss.say(["I can not reach my target position because my path or target is blocked, I will abort."],False)
                    rospy.wait_for_service('base_controller/stop',10)
                    try:
                        stop = rospy.ServiceProxy('base_controller/stop',Trigger)
                        resp = stop()
                    except rospy.ServiceException, e:
                        error_message = "%s"%e
                        rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                    return 'failed'
                else:
                    timeout = timeout + 1
                    rospy.sleep(1)
            else:
                timeout = 0
        return 'failed'
        

## Select grasp state
#
# This state select a grasping strategy. A high object will be grasped from the side, a low one from top.
class select_grasp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['top', 'side', 'failed'],
            input_keys=['object_pose'])
        
        self.height_switch = 0.8 # Switch to select top or side grasp using the height of the object over the ground in [m].

    def execute(self, userdata):
        return 'top'


## Grasp side state
#
# This state will grasp an object with a side grasp
class grasp_side(smach.State):

    def __init__(self, max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'retry', 'no_more_retry', 'failed'],
            input_keys=['object'])
        
    def execute(self, userdata):
        return 'failed'


## Grasp top state
#
# This state will grasp an object with a top grasp
class grasp_top(smach.State):

    def __init__(self, max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'retry', 'no_more_retry', 'failed'],
            input_keys=['object'])


    def execute(self, userdata):
        return 'failed'        

## General grasp
#  
# This state will grasp an object based on the configuration passed
class grasp_general(smach.State):

    def __init__(self, max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'retry', 'failed'],
            input_keys=['object'],
            output_keys=['grasp_conf'])


    def execute(self, userdata):
        return 'failed'     

## Open door state
#
# This state will open a door
class open_door(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['door_pose'])

    def execute(self, userdata):
        #TODO teach hinge and handle position relative to the door_pose (this means: detected ipa_logo)
        return 'succeeded'
    
## Put object on tray state
#
# This state puts a grasped object on the tray
class put_object_on_tray(smach.State):

    def __init__(self):
        smach.State.__init__(cob_object_detection,
            self,
            outcomes=['succeeded', 'failed'])

    def execute(self, userdata):
        #TODO select position on tray depending on how many objects are on the tray already. This has to be counted by this state itself
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

        self.object_list = DetectionArray()
        self.max_retries = max_retries
        self.retries = 0
        self.object_name = object_name
        
        self.torso_poses = []
        self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left_extreme")

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
            handle_arm.wait()
            handle_head.wait()
            handle_torso.wait()
        handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) # have an other viewing point for each retry
        
        # move sdh as feedback
        sss.move("sdh","home",False)
    
        # check if object detection service is available
        try:
            rospy.wait_for_service('/object_detection/detect_object',10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            self.retries = 0 # no object found within min_dist start value
            return 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
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
        if obj.header.frame_id == "":
            self.retries += 1
            return 'retry'

        #check if label of object fits to requested object_name
        if obj.label != object_name:
            sss.say(["The object name doesn't fit."],False)
            self.retries += 1
            return 'retry'

        # we succeeded to detect an object
        userdata.object = obj
        self.retries = 0
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
