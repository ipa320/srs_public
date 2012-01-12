#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making_experimental')
import rospy
from std_msgs.msg import String

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the SRS DM action, including the
# goal message and the result message.
import srs_decision_making_experimental.msg as xmsg

def DM_client():
    
    # Creates a goal to send to the action server.
    _goal=xmsg.ExecutionGoal()
    _goal.action="get"
    _goal.parameter="milk"
    _goal.priority=0
    
    
    pub_goal = rospy.Publisher('/srs_decision_making_actions/goal', String)
    
    return pub_goal.publish("[goal: get, parameter: milk, prioirty: 0]")
    
    
    

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dm_client')
        result = DM_client()
        rospy.loginfo('result %s',result)
        # print ("Result:" result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
