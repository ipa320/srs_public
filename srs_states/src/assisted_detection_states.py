import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
import smach_ros

from math import *
import copy
from struct import *

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from shared_state_information import *
from eval_objects import *
from cob_3d_mapping_msgs.msg import *

## Detect state
#
# This state will try to detect an object.


class detect_object_assited(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','failed','preempted'],
            input_keys=['object_name'],
            output_keys=['object_list'])

        self.object_list = DetectionArray()
        self.max_retries = max_retries
        self.retries = 0
        self.object_name = object_name
        self.srv_name_object_detection = '/object_detection/detect_object'

        self.torso_poses = []
        self.torso_poses.append("back_right")
        self.torso_poses.append("back")
        self.torso_poses.append("back_left")
        self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left_extreme")
        #self.listener = tf.TransformListener()
        self.the_object = ''
        self.the_object_pose = ''


    def execute(self, userdata):
        pass



class user_intervention_on_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'bb_move', 'give_up', 'failed', 'preempted'],
                             input_keys = ['target_object_name', 'object_list'],
                             output_keys=['object','object_pose','bb_pose'])
        
    def execute(self, userdata):
        pass