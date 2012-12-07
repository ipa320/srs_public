#!/usr/bin/env python

import roslib
roslib.load_manifest('srs_scenarios')
import rospy

import rosbag
from std_msgs.msg import Int32, String

import tf
from tf.msg import *
from tf.transformations import euler_from_quaternion

from simple_script_server import *
sss = simple_script_server()

from kinematics_msgs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import *
from move_base_msgs.msg import *

import math
import rostopic

import os
import sys, subprocess

import itertools

class tf_aggregator():

    def __init__(self):
        
        self.transforms = {}
        
if __name__=="__main__":

    rospy.init_node('joint_test')
    tfa = tf_aggregator()
    
    while not rospy.is_shutdown():
        
        msg =  rospy.wait_for_message("/tf", tfMessage)
        
        for trans in msg.transforms:
            frame_id = trans.header.frame_id
            child_frame_id = trans.child_frame_id
            tfa.transforms[child_frame_id] = frame_id
            rospy.loginfo(tfa.transforms)
        rospy.loginfo(msg)
        
        rospy.sleep(2)
