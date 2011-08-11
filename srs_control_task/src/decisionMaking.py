#! /usr/bin/env python
#################################################################################
# SRS project - decisionMaking.py                                               #
# Place : Manufacturing Engineering Centre, Cardiff University                  #
# Author : Damien LEBON, Sylvain Chardonneau & Gwendal LOUBES                   #
# Date : August 2011                                                            #
#-------------------------------------------------------------------------------#
# Description : Allows to make the links between the different services. So in  #
# first it receives  the command by human operator, then, call the translation  #
# service, then, the task execution service. Finally, it return result and      #
# feedback about all the sequence to the user.                                  #
#################################################################################

import roslib; roslib.load_manifest('srs_control_task')
import rospy
import actionlib

import sys
import time


import srs_control_task.msg as xmsg
import srs_control_task.srv as xsrv
from srs_control_task.srv import *

class DecisionMaking:
    
    #### Initialization and declaration of the variables ####
    def __init__(self):
          self._feedbackUser = xmsg.UserFeedback()
          self._resultUser = xmsg.UserResult()
          self.serverUser = actionlib.SimpleActionServer(rospy.get_name(), xmsg.UserAction, self.getCommandUser, False)
          self.serverUser.start()
          rospy.loginfo("Waiting for a new task ...")
	  rospy.Service('message_errors', errors, self.handle_message_errors)

    #### Display the feedback for the user ####
    def execute_callback(self,fb):
    	print ('Display feedback')
    	rospy.loginfo("%s",fb.current_state)
	rospy.loginfo("%s",fb.solution_required)
    	if(fb.solution_required == True):
            rospy.loginfo("Solution required")
            #self.DecisionMaking_errors_server()
            ##lancement du server erreur
            rospy.loginfo("%s",fb.current_state)
		
    #### Allows to define the type of the error during an exceptional case ####
    def handle_message_errors(self,req):
    	print "Returning [%s]"%(req.current_state)
        print req.exceptional_case_id
        rospy.loginfo("Enter a new position to solve problem :")
        self.solfromUser = raw_input()
	return errorsResponse(self.solfromUser)

    #### Allows to translate the action and the parameter send by the user ####
    def translate_client(self, wordToTranslate):
        rospy.wait_for_service('translate')
        try:
            translate = rospy.ServiceProxy('translate', xsrv.translation)
            wordTranslated = translate(wordToTranslate)
            #print 'action : %s with parameter :'%(actionTranslated, parameterTranslated)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        return wordTranslated
    
    #### the client of the actionlib between the decision making and the executor service ####
    def execute_task(self, action_to_execute, param_to_execute):
        self._feedbackUser.currentState = 'Executing'
        self.serverUser.publish_feedback(self._feedbackUser)
        
        self.exec_task_client = actionlib.SimpleActionClient('task_executor', xmsg.ExecutionAction)  #param: serverName, actionnameAction
        self.exec_task_goal = xmsg.ExecutionGoal()
        
        print "the task to execute is : %s to %s"%(action_to_execute[10:],param_to_execute[10:])
        rospy.loginfo("Waiting for executing server ...")
        self.exec_task_client.wait_for_server()
        
        self.exec_task_goal.action = action_to_execute[10:]
        self.exec_task_goal.parameter = param_to_execute[10:]
        
        self.exec_task_client.send_goal(self.exec_task_goal, feedback_cb = self.execute_callback)
            
        self.exec_task_client.wait_for_result()
        #####show the result######
        rospy.loginfo("Result is : %s",self.exec_task_client.get_result())
        #int_result = self.exec_task_client.get_result()[13:]
        #print float(int_result)
        self._resultUser.returnValue = 2
        self.serverUser.set_succeeded(self._resultUser)
    
    
    #### The function allows to call the translator service to get the goal needed for the executor service ####
    def translate(self, goal_to_translate):
        self._feedbackUser.currentState = 'Translating'
        self.serverUser.publish_feedback(self._feedbackUser)
        
        actionTranslated = self.translate_client(goal_to_translate.actionName)
        paramTranslated = self.translate_client(goal_to_translate.parameter)
        
        #Call the executor service
        self.execute_task(actionTranslated.__str__(), paramTranslated.__str__())
        
    #### This function realize a little bit process to get the command from the user and send it to the translator ####
    def getCommandUser(self, usergoal):
        self._feedbackUser.currentState = 'Getting command'
        self._feedbackUser.solutionRequired = True
        self._feedbackUser.exceptionalCase_id = 1
        self._resultUser.returnValue=2 
        self.serverUser.publish_feedback(self._feedbackUser)
        
        rospy.loginfo('current goal : \n%s',usergoal)
        self.translate(usergoal)

        
       
if __name__ == '__main__':
  rospy.init_node('control_task_server')
  server = DecisionMaking()
  rospy.spin()
