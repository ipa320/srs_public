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

class joint_state_aggregator():

    def __init__(self):
        
#        self.effort = []
        self.position = []
        self.velocity = []
        self.names = []
        self.jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState, 10)
        
        for a,b,c in itertools.izip(self.jointsMsg.name, self.jointsMsg.position,self.jointsMsg.velocity):
            self.names.append(a)
            self.velocity.append(b)
            self.position.append(c)
        
    def process_joints(self):
    
        self.jointsMsg = rospy.wait_for_message("/joint_states", sensor_msgs.msg.JointState, 10)
        self.temp_name = self.jointsMsg.name
        self.temp_vel = self.jointsMsg.velocity
        self.temp_pos = self.jointsMsg.position
        
        for a,b,c in itertools.izip(self.jointsMsg.name, self.jointsMsg.position,self.jointsMsg.velocity):
        
            if(a) not in self.names:
                self.names.append(a)
                self.position.append(b)
                self.velocity.append(c)
                
            self.position[self.names.index(a)] = self.temp_pos[self.temp_name.index(a)]
        
        self.jointsMsg.name = self.names
        self.jointsMsg.position = self.position
        self.jointsMsg.velocity = self.velocity
        
        return self.jointsMsg
        
if __name__=="__main__":

    rospy.init_node('joint_test')
    ja = joint_state_aggregator()
    
    while not rospy.is_shutdown():
        
        ja.process_joints()
        
        rospy.loginfo("Current message:")
        rospy.loginfo(ja.jointsMsg)
        rospy.loginfo("Collected names:")
        rospy.loginfo(ja.names)
        rospy.loginfo(ja.position)
        rospy.loginfo(ja.velocity)
        
        rospy.sleep(2)
