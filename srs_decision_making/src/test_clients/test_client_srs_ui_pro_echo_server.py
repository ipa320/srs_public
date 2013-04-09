#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making')
import rospy
import actionlib
import smach
import smach_ros
import simplejson as json

#import states within srs_decision_making
import srs_ui_pro.msg as echo_server_msg

from std_msgs.msg import String

def test_client_srs_ui_pro_echo_server(json_input):
    client = actionlib.SimpleActionClient('srs_ui_pro/echo_server', echo_server_msg.dm_serverAction)
    client.wait_for_server()
    
    goal = echo_server_msg.dm_serverGoal()
    goal.json_input = json_input
    print "###type of goal is", type(goal)
    print "###goal is", goal
    print "###goal.json_input is", goal.json_input 
    
    #server_feedback = echo_server_msg.dm_serverFeedback()
    #server_result = echo_server_msg.dm_serverResult()
    server_feedback = echo_server_msg.dm_serverFeedback()
    server_result = echo_server_msg.dm_serverResult()
    try:
        client.send_goal(goal, result_callback, None, feedback_callback)
    except rospy.ServiceException, e:
        print "Service did not process request: %s"%str(e)
    
    rospy.spin()#,,,,,,,,,,,,,,,,,,
    

def feedback_callback(server_feedback):
    if server_feedback is not None:
        #rospy.loginfo ("server_feedback is: %s", server_feedback)
        #rospy.loginfo ("server_feedback.current_status is: %s", server_feedback.current_status)
        rospy.loginfo ("server_feedback.json_feedback is: %s",server_feedback.json_feedback)
    else: 
        print "### no feedback from echo server..."

def result_callback(state, server_result):
     if server_result is not None:
        #rospy.loginfo ("server_feedback is: %s", server_feedback)
        #rospy.loginfo ("server_feedback.current_status is: %s", server_feedback.current_status)
        rospy.loginfo ("state is: %s",state)
        rospy.loginfo ("server_result.json_result is: %s",server_result.json_result)
        
        server_json_result = server_result.json_result
        json_decoded = json.loads(server_json_result)
        result = json_decoded['result']
        # result should be succeeded
        if result == "succeeded":
            print  'result is completed'
        elif result == "failed":
            return "result is failed"
        else:
            return "result is give_up"
     else: 
        print "### no feedback from echo server..."
        
if __name__ == '__main__':
    try:
        global current_task_info   
        rospy.init_node('srs_ui_pro_echo_server_client')
        #json_input = '{"exception_id":33, "tasks":[{"time_schedule":1263798000000,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}, {"time_schedule":2222222222,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}], "additional_information":"The robot needs the user to assist in completing the task of moving to [0 1 3.14]"}'
        json_input = "{\"exception_id\":33, \"tasks\":[{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"pose2d_string\":\"[0 1 3.14]\"}}, {\"time_schedule\":2222222222,\"task\":\"move\",\"destination\":{\"pose2d_string\":\"[0 1 3.14]\"}}], \"additional_information\":\" ****balabla The robot needs the user to assist in completing the task of moving to [0 1 3.14]\"}"
        #json_input = '{goal: {json_input: "{"exception_id":33, "tasks":[{"time_schedule":1263798000000,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}, {"time_schedule":2222222222,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}], "additional_information":"The robot needs the user to assist in completing the task of moving to [0 1 3.14]"}"}}'
        test_client_srs_ui_pro_echo_server(json_input)
        # result format is: {"exception_id":1, "feedback_type ":"unstructured", "content":"Waiting srs_ui_pro"}
        # to restructure the result, and publish it as current_task_info
        #feedback = xmsg.ExecutionFeedback()
       #current_task_info._srs_as._as.publish_feedback(result)
        
    except rospy.ROSInterruptException:
        print "error before completion"