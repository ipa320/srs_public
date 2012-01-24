#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_decision_making_interface')

from srs_decision_making_interface.srv import srs_dm_ui_interface
from std_msgs.msg import *
import rospy
import actionlib

def srs_decision_making_ui_interface_client(action, parameter, priority):

    rospy.wait_for_service('srs_decision_making_ui_interface')
    
    srs_decision_making_action = rospy.ServiceProxy('srs_decision_making_ui_interface', srs_dm_ui_interface)

    resp = srs_decision_making_action(action, parameter, priority)
        
    return resp
    
if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dm_client_test')
        action = 'get'
        parameter = 'Milkbox'
        priority = '1'
        
        print (action, parameter, priority)
        result = srs_decision_making_ui_interface_client(action, parameter, priority)
        rospy.loginfo('The task has been passed to DM, the allocated task id is: %s',result)
        # print ("Result:" result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
