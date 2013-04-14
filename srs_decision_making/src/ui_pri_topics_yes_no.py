#!/usr/bin/python

#### this program publish service /answer_yes_no (parameter is the question for the user) and deals with the clients through topics /DM_UI/interface_cmd_yes_no (for the question) and /DM_UI/interface_cmd_yes_no_response (for the answer)

# ROS imports

import roslib; roslib.load_manifest('srs_decision_making')
import rospy
import srs_decision_making.srv as xsrv
import srs_msgs.msg as srs_msg
from random import randrange

class UI_PRI_TOPICS_YES_NO:
    
    #### Initialization and declaration of the variables ####
    def __init__(self):
        rospy.loginfo("Public topics for UI_PRI_OBJ_SEL ...")
        self.pubUIobjsel = rospy.Publisher('DM_UI/interface_cmd_yes_no',srs_msg.DM_UIobjcom)
        self.user_answer =""
        self.user_comobj_id = 0
        rospy.Subscriber ("DM_UI/interface_cmd_yes_no_response",srs_msg.UI_DMobjresp, self.callbackUIobj)

    ### Declaration of callback function for the object selection commands from the user interface through the topic
    def callbackUIobj (self,data):
        rospy.loginfo ("I heard %s %i from the UI_PRI",data.solution, data.responseID)
        if (data.responseID == self.user_comobj_id):
            self.user_answer = data.solution     
            rospy.loginfo ("Match between responseID and requestID. Now user_answer is:%s ",self.user_answer)
        else:
            print ("unfortunately the responceID:%i does not correspond to requestId:%i" % (data.responseID,self.user_comobj_id))
            self.user_answer = ""

### the handler for the service call    
    def selectObjHandle(self,req):

        rospy.loginfo("The user will be invited to select a new object:")

        satisfaction_flag = False
        
        self.time_out_max = 20
        try:
            self.time_out_max = rospy.get_param("srs/common/max_time_out")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default value for max time_out")
        
        while (not satisfaction_flag):
            self.user_answer = ""
            self.user_comobj_id = randrange(1000)
            message1 = srs_msg.DM_UIobjcom ("ask",req.message,self.user_comobj_id)
            self.pubUIobjsel.publish(message1)
            rospy.loginfo("Message sent to user to ask Yes or No. RequestId is:", )
            
            timeout=0
            while (self.user_answer =="" and timeout < self.time_out_max):
                print "waiting for response",timeout," seconds from the user"
                timeout = timeout + 1
                print "user_answer is :",self.user_answer
                rospy.sleep(1)

                      
            print "the final self.user_answer is.:",self.user_answer


            if (self.user_answer.strip()=="y" or self.user_answer.strip()=="Y"or self.user_answer.strip()=="yes"or self.user_answer.strip()=="yes" or self.user_answer.strip()=="Yes" or self.user_answer.strip()=="YES"):
                rospy.loginfo("The response from the user is: <<Yes>>.")

                self.user_respobjsol =""
                self.user_respobjpar = ""
                self.giveup = 0
                self.answer = "Yes"
                satisfaction_flag = True
            
            else:    

                if (self.user_answer.strip()=="n" or self.user_answer.strip()=="N"or self.user_answer.strip()=="no" or self.user_answer.strip()=="No" or self.user_answer.strip()=="NO"):
                   rospy.loginfo("The response from the user is: <<No>>.")
                   satisfaction_flag = True
                   self.giveup = 0
                   self.answer = "No"

                
                else: 
                   rospy.loginfo("There was no responce from the user or it didn't make sense... Assuming NO")
                   satisfaction_flag = True
                   self.giveup = 1
                   self.answer = "No"



                      
        return xsrv.answer_yes_noResponse(self.answer,self.giveup)
    
if __name__ == '__main__':
    rospy.init_node('control_task_server')
    UI_PRI_OBJ_TOPIC_s = UI_PRI_TOPICS_YES_NO()
    rospy.Service('answer_yes_no', xsrv.answer_yes_no, UI_PRI_OBJ_TOPIC_s.selectObjHandle)
    rospy.spin()
