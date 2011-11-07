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

    def execute(self, userdata):
        rospy.loginfo("target base pose: %s", userdata.base_pose)
        self.counter=self.counter+1
        rospy.sleep(1)
        if self.counter>1:
            return 'succeeded'
        else:
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
        smach.State.__init__(
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
        rospy.loginfo("target object is: %s", userdata.object_name)
        rospy.sleep(1)
        # we succeeded to detect an object
        userdata.object = (0,0,0,0,0,0,0)
        self.retries = 0
        return 'succeeded'

