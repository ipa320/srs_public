#!/usr/bin/python
# ROS imports


import roslib; roslib.load_manifest('srs_decision_making')
import rospy
import srs_decision_making.srv as xsrv
import srs_msgs.msg as srs_msg

class UI_PRI_TOPICS:
    
    #### Initialization and declaration of the variables ####
    def __init__(self):
        rospy.loginfo("Public topics for UI_PRI ...")
        self.pubUIerr = rospy.Publisher('DM_UI/interface_cmd',srs_msg.DM_UIcom)
        self.user_respsol =""
        self.user_resppar =""
        self.user_com_id = 0
        rospy.Subscriber ("DM_UI/interface_cmd_response",srs_msg.UI_DMresp, self.callbackUI)

    ### Declaration of  callback function for the commands from the user interface
    def callbackUI (self,data):
        rospy.loginfo ("I heard %s %s %i from the UI_PRI",data.solution,data.parameter, data.responseID)
        if (data.responseID == self.user_com_id):
            self.user_respsol = data.solution 
            self.user_resppar = data.parameter
            rospy.loginfo ("Match between responseId and requestID. Now user_respsol is:%s and user_resppar is:%s",self.user_respsol,self.user_resppar)  
        else:
            print ("but the id:%s does not correspond to requestId:%s",self.user_com_id)

    
    def handle_message_errors(self,req):
        print "Returning [%s]"%(req.current_state)
        print req.exceptional_case_id
        rospy.loginfo("The user will be invited to enter a new position to solve problem :")
        self.solfromUser = xsrv.errorsResponse()
        satisfaction_flag = False
        
        self.time_out_max = 30  
        try:
            self.time_out_max = rospy.get_param("srs/common/max_time_out")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default value for max time_out") 
        
        while (not satisfaction_flag):
            rospy.loginfo("Message sent to user to ask if he wants to continue:")
            self.user_respsol = ""
            message1 = srs_msg.DM_UIcom ("continue?","There is an obstacle on the robot's path. Do you want to try to move the robot manually? ",5)
            self.user_com_id = 5
            self.pubUIerr.publish(message1)
            
            timeout=0
            while (self.user_respsol =="" and timeout < self.time_out_max):
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
                while (self.user_respsol =="" and timeout < 3*self.time_out_max ):
                  print "waiting for response for",timeout," seconds from the user"
                  print "user_respsol is :",self.user_respsol
                  timeout = timeout + 1
                  rospy.sleep(1)
                  
                if (self.user_respsol.strip() =="move"):
                    satisfaction_flag = True
                    self.solfromUser.giveup = 0
                    self.solfromUser.solution = self.user_resppar.strip()
                    print "We got a move response from the user"
                    print(self.solfromUser)
                else:
                    satisfaction_flag = False
                    rospy.loginfo("The user response doesn't make sense or timed out")
                    self.solfromUser.giveup = 1
                    self.solfromUser.solution = ""

            else:
                rospy.loginfo("The response from the user is: <<Do not to continue>>. Giving up")
                satisfaction_flag = True
                self.solfromUser.giveup = 1
                self.solfromUser.solution = ""

                print(self.solfromUser)        
                

        return xsrv.errorsResponse(self.solfromUser.solution,self.solfromUser.giveup)
    
if __name__ == '__main__':
    rospy.init_node('control_task_server')
    UI_PRI_TOPICs = UI_PRI_TOPICS()
    rospy.Service('message_errors', xsrv.errors, UI_PRI_TOPICs.handle_message_errors)
    rospy.spin()