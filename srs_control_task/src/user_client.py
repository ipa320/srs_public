#! /usr/bin/env python
#################################################################################
# SRS project - user_client.py                                                  #
# Place : Manufacturing Engineering Centre, Cardiff University                  #
# Author : Damien LEBON, Sylvain Chardonneau & Gwendal LOUBES                   #
# Date : August 2011                                                            #
#-------------------------------------------------------------------------------#
# Description : This the Actionlib client of the user service. He let the human #
# operator to create a command which is send as a goal to the decision making.  #
# Thanks to this code, the user can receive the result and the feedback  of all #
# the sequence to execute a task                                                #
#################################################################################

import roslib; roslib.load_manifest('srs_control_task')
import rospy
import actionlib

from srs_control_task.msg import *


def execute_callback(fb):
    print ('Current feedback')
    if(fb.solutionRequired == True):
        rospy.loginfo("%s",fb.currentState)
        

def execute_client():
    #Wait a connection with the server
    client = actionlib.SimpleActionClient('control_task_server', UserAction)
    goal = UserGoal()
    goal.solution = 'notNecessary'
    rospy.loginfo("Waiting for server ...")
    client.wait_for_server()
    
    #The user creates his command
    print "Which action ? "
    goal.actionName = raw_input()#"move"
    print "Which parameter for this action ? "
    goal.parameter = raw_input()#"kitchen"
    
    #Sends goal to the server
    client.send_goal(goal, feedback_cb = execute_callback)
    
    #Exceptional case need a solution 
    if (srs_control_task.msg.UserFeedback().solutionRequired == True):
            print "Which parameter to solve the problem ? "
            goal.solution = raw_input()#"[1, 2, 3]"
            client.send_goal(goal)
            
    else:
        #out = True
        client.wait_for_result()
        #####show the result######
        rospy.loginfo("Result is : %s",client.get_result())


if __name__ == '__main__':
    rospy.init_node('user_client_py')
    execute_client()

