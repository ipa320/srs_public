#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making')
import rospy
import sys

import actionlib

import srs_decision_making.msg as xmsg

def DM_client(json_parameters):
    client = actionlib.SimpleActionClient('srs_decision_making_actions', xmsg.ExecutionAction)

    client.wait_for_server()

    # Creates a goal to send to the action server.
    _goal=xmsg.ExecutionGoal()
    #_goal.action="move"
    #_goal.parameter=target_pos
    #_goal.priority=0

    _goal.json_parameters = json_parameters
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
        #json_parameters = '{"tasks":[{"time_schedule":1263798000000,"task":"get","deliver_destination":{"predefined_pose":"order"},"object":{"object_type":"Milkbox"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}'
        #json_parameters = '{"tasks":[{"time_schedule":1263798000000,"task":"get","deliver_destination":{"pose2d":{"theta":0.0,"y":0.0,"x":0.0}},"object":{"object_type":"Milkbox"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}'
        #json_parameters = '{"tasks":[{"task":"move", "mode":"manual", "component":"torso","destination":{"predefined_pose":"nod"}}],"initializer":{"device_type":"ui_pri","device_id":"ui_pri_101"}} '
        json_parameters = '{"tasks":[{"task":"move", "mode":"manual", "component":"torso","destination":{"torso_pose":{"tilt1":-0.1,"pan":0.1,"tilt2":0.15}}}],"initializer":{"device_type":"ui_pri","device_id":"ui_pri_101"}}'

        #json_parameters = "{\"tasks\":[{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"predefined_pose\":\"home\"}}],\"initializer\":{\"device_type\":\"ui_loc\",\"device_id\":\"ui_loc_0001\"}}"
        rospy.init_node('dm_client2')
        result = DM_client(json_parameters)
        rospy.loginfo('result %s',result)
        # print ("Result:" result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
