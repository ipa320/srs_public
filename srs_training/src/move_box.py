#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_training')
import rospy
from geometry_msgs.msg import Twist
import tf

def callback(msg):
    print msg
    # get TF
    # add step based on velocity and time
    # publish TF
    
    

def move_box():
    rospy.init_node('move_box')
    rospy.Subscriber("/base_controller/command", Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    move_box()
