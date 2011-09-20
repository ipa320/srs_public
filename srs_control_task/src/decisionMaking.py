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
#import errors.srv as err_msg
from srs_control_task.srv import *

#from srs_msgs import dm_uicom.msg
#from dm_uicom.msg import dm_uicom
#from srs_msgs import dm_uicom as dm_ui

import srs_msgs.msg as srs_msg



from std_msgs.msg import String, Bool, Int32

class DecisionMaking:
    
    #### Initialization and declaration of the variables ####
    def __init__(self):
        self._feedbackUser = xmsg.UserFeedback()
        self._resultUser = xmsg.UserResult()
        self.solfromUser = errorsResponse()
        self.serverUser = actionlib.SimpleActionServer(rospy.get_name(), xmsg.UserAction, self.getCommandUser, False)
        self.serverUser.start()
        rospy.loginfo("Waiting for a new task ...")
	rospy.Service('message_errors', errors, self.handle_message_errors)
        self.pubUIerr = rospy.Publisher('DM_UI/interface_cmd',srs_msg.DM_UIcom)
        self.user_respsol =""
        self.user_resppar =""
        self.user_com_id = 0
        rospy.Subscriber ("DM_UI/interface_cmd_response",srs_msg.UI_DMresp, self.callbackUI)

    ### Declaration of  callback function for the commands from the user interface
    def callbackUI (self,data):
        rospy.loginfo ("I heard %s %s %i from the UI_PRI",data.solution,data.parameter,data.responseID)
        if (data.responseID == self.user_com_id):
            self.user_respsol = data.solution 
            self.user_resppar = data.parameter
            rospy.loginfo ("Match between responseId and requestID. Now user_respsol is:%s and user_resppar is:%s",self.user_respsol,self.user_resppar)  
        else:
            print ("but the id:%s does not correspond to requestId:%s",self.user_com_id,data.responseID)

    


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
        rospy.loginfo("The user will be invited to enter a new position to solve problem :")
        self.solfromUser = errorsResponse()
        satisfaction_flag = False
        while (not satisfaction_flag):
            rospy.loginfo("Message sent to user to ask if he wants to continue:")
            self.user_respsol = ""
            message1 = srs_msg.DM_UIcom ("continue?","There is an obstacle on the robot's path. Do you want to try to move the robot manually? ",5)
            self.user_com_id = 5
            self.pubUIerr.publish(message1)
            
            timeout=0
            while (self.user_respsol =="" and timeout < 30):
                print "waiting for response",timeout," seconds from the user"
                timeout = timeout + 1
                print "user_respsol is :",self.user_respsol
                rospy.sleep(1)

                      
            print "user_respsol is.:",self.user_respsol


            if (self.user_respsol.strip()=="continue"):
                rospy.loginfo("Just received continue from the user and will ask for a new position to solve the problem :")
                self.user_respsol =""
                self.user_resppar = ""
                message1 = srs_msg.DM_UIcom ("position?","Plese specify a new intermedeate position for the robot to try reaching the target from there",6)
                rospy.loginfo("Just sent a request to the user to give me a new position to solve the problem :")
                self.user_com_id = 6
                self.pubUIerr.publish(message1)
                #t2= raw_input()              
                
                timeout=0
                while (self.user_respsol =="" and timeout < 90):
                  print "waiting for response for",timeout," seconds from the user"
                  print "user_respsol is :",self.user_respsol
                  timeout = timeout + 1
                  rospy.sleep(1)
                  
                if (self.user_respsol.strip() =="move"):
                    satisfaction_flag = True
                    self.solfromUser.giveup = 0
                    self.solfromUser.solution = self.user_resppar.strip()
                    print "We got response from the user"
                    print(self.solfromUser)
                else:
                    satisfaction_flag = False
                    rospy.loginfo("The user response doesn't make sence or timed out")
                    self.solfromUser.giveup = 1
                    self.solfromUser.solution = ""

            else:
                rospy.loginfo("The response from the user is: <<Do not to continue>>. Giving up")
                satisfaction_flag = True
                self.solfromUser.giveup = 1
                self.solfromUser.solution = ""

                print(self.solfromUser)        
                

        return errorsResponse(self.solfromUser.solution,self.solfromUser.giveup)
        
    """
        print "Returning [%s]"%(req.current_state)
        print req.exceptional_case_id
        t1=""
        self.solfromUser = errorsResponse()
        while (not (t1.lower() in ("yes", "no"))):
            rospy.loginfo("Do you want to continue (yes or no) :")
            t1= raw_input()
            
        if (t1.lower() in ("yes")):
            rospy.loginfo("Enter a new position to solve problem :")
            t2= raw_input()              
        #   self.solfromUser.giveup = 0
            self.solfromUser.solution = t2
            print(self.solfromUser)
        else:
            rospy.loginfo("Give up")
        #    self.solfromUser.giveup = 1
            self.solfromUser.solution = ""
            print(self.solfromUser)
            
	    return errorsResponse(self.solfromUser.solution)
    """
   

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
        
        
        if (self.exec_task_client.get_result() == 3):
            self.serverUser.set_succeeded(self._resultUser)
        else:
            self.serverUser.set_preempted(self._resultUser)
 #           self.serverUser.set_succeeded(4)
    
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
        self._resultUser.returnValue=3 
        self.serverUser.publish_feedback(self._feedbackUser)
        
        rospy.loginfo('current goal : \n%s',usergoal)
        self.translate(usergoal)

        
       
if __name__ == '__main__':
  rospy.init_node('control_task_server')
  server = DecisionMaking()
  rospy.spin()
