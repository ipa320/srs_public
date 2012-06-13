#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making')
import rospy
import sys

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the SRS DM action, including the
# goal message and the result message.
import srs_decision_making.msg as xmsg

def DM_client(target_pos):
    # Creates the SimpleActionClient, passing the type of the action
    # constructor.
    client = actionlib.SimpleActionClient('srs_decision_making_actions', xmsg.ExecutionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    _goal=xmsg.ExecutionGoal()
    _goal.action="move"
    _goal.parameter=target_pos
    _goal.priority=0
    _goal.json_parameters = "{\"tasks\":[{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"predefined_pose\":\"home\"}}],\"initializer\":{\"device_type\":\"ui_loc\",\"device_id\":\"ui_loc_0001\"}}"
    # Sends the goal to the action server.
    client.send_goal(_goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        target_pos = sys.argv[1]
        print "moving to", target_pos
        rospy.init_node('dm_client2')
        result = DM_client(target_pos)
        rospy.loginfo('result %s',result)
        # print ("Result:" result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
