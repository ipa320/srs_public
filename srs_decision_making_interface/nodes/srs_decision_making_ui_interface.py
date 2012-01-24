#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_decision_making_interface')

from srs_decision_making_interface.srv import srs_dm_ui_interface
import srs_decision_making_interface.msg as xmsg
from std_msgs.msg import *
import rospy
import actionlib

#client = ''

def srs_decision_making_ui_interface_handle(req):
    global client
    _goal=xmsg.srs_actionGoal()
    _goal.action=req.action
    _goal.parameter=req.parameter
    _goal.priority=int(req.priority)
    client.send_goal(_goal)    
    return client.gh.comm_state_machine.action_goal.goal_id.id

def srs_decision_making_ui_interface():

    s = rospy.Service('srs_decision_making_ui_interface', srs_dm_ui_interface, srs_decision_making_ui_interface_handle)
    print "Ready to translate request from UI to ROS network."
    rospy.spin()



if __name__ == "__main__":
    rospy.init_node('srs_decision_making_ui_interface')
    global client
    client = actionlib.SimpleActionClient('srs_decision_making_actions', xmsg.srs_actionAction)
    client.wait_for_server()
    srs_decision_making_ui_interface()