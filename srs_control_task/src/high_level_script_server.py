#!/usr/bin/env python
#################################################################################
# SRS project - high_level_script_server.py                                     #
# Place : Manufacturing Engineering Centre, Cardiff University                  #
# Author : Damien LEBON, Sylvain Chardonneau & Gwendal LOUBES                   #
# Date : August 2011                                                            #
#-------------------------------------------------------------------------------#
# Description : Supply all the states use in the Smach state machine to         #
# execute a task.                                                               #
# Also contains the error manager. If an error occured, the script ask the user #
# a solution to solve the problem. Then, the task is resumed.
#################################################################################

# ROS imports
import roslib; roslib.load_manifest('srs_control_task')

import rospy
import smach
import smach_ros

from std_msgs.msg import String, Bool, Int32
from cob_srvs.srv import Trigger

import time
import tf

import actionlib

# include script server, to move the robot
from simple_script_server import simple_script_server
sss = simple_script_server()

# msg imports
from geometry_msgs.msg import *
import srs_control_task.msg as xmsg
import srs_control_task.srv as xsrv
from srs_control_task.srv import *

hdl_torso = ""
hdl_tray = ""
hdl_arm = ""
hdl_sdh = ""
hdl_head = ""
hdl_base = ""

#current_action = ""


#------------------- Init section -------------------#
# Description : this is a smach state which initialize the components before running the task
# input keys : 'action_req' contains the name of the action required by the decision making
# output keys : None
# outcomes : 'trigger' launch the next state if the initialization succeeded.
#            'init_error' goto the error manager if an error occured during the
#                           components initialization
class INIT_COMPONENTS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trigger'],
                             input_keys=['action_req'])
        
        global hdl_torso
        global hdl_tray
        global hdl_arm
        global hdl_sdh
        global hdl_head
        global hdl_base
        self.current_action=""
#        global current_action
        #self.sc = script()
        #self.sc.Start()
        #self.sss = self.sc.sss
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.count = 0

    def execute(self, userdata):
        
        # publish the feedback current_state to the decision making
        while not rospy.is_shutdown():
            self.pub_fb.publish("Init Components is running ...")
            rospy.sleep(1.0)
            #if (self.count <1):
            #    self.current_action = "INIT_COMPONENTS"
            #else:
            #    if (self.current_action != "INIT_COMPONENTS"): 
            #        return 'trigger'
            #self.count += 1
            #if (self.count == 1):
            #    break
            
            rospy.loginfo("Init all position components is running") 
        
        # Init depends on the action required
            if (userdata.action_req == "move"):
                # move to initial positions
                # hdl_torso = sss.move("torso", "home", False)
                # hdl_tray = sss.move("tray", "down", False)
                # hdl_arm = sss.move("arm", "folded", False)
                # hdl_sdh = sss.move("sdh", "cylclosed", False)
                # hdl_head = sss.move("head", "back", False)
            
                # wait for initial movements to finish
                # hdl_torso.wait()
                # hdl_tray.wait()
                # hdl_arm.wait()
                # hdl_sdh.wait()
                # hdl_head.wait()
            
                # if (hdl_arm.get_state() == 3):#ARM folded succeeded
                return 'trigger'
            #else:
                #    return 'init_error'
        
            elif (userdata.action_req == "grasp"):
                rospy.loginfo("Not initialization components for action GRASP")
                return 'trigger'
            
            else:
                rospy.loginfo("components initialization not necessary")
                return 'trigger'
        
       # return 'init_error'
        
#------------------- SELECT section -------------------#
# Description : this Smach state is used to go to the required action.
#                If the task is resumed, it will be done from here. 
# input keys : 'action_required' contains which action is required by the decision making.
# output keys : None
# outcomes : 'goto_move', 'goto_detect', 'goto_grasp'
class SELECT(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['goto_move',
                                             'goto_detect',
                                             'goto_grasp',
                                             'nok'],
                             input_keys=['action_required'],
                             output_keys=['action_selected']
                             )
        
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.pub_fb2 = rospy.Publisher('fb_executing_solution', Bool)
        self.count = 0
        self.current_selection = ""
       # global current_action
        
    def execute(self,userdata):
        # publish the feedback current_state to the decision making
        while not rospy.is_shutdown():
            self.pub_fb.publish("SELECT the action ...")
            rospy.sleep(1.0)
            self.count += 1
            self.pub_fb2.publish(False)
            rospy.sleep(1.0)
            #if (self.count >=1):
            #    break
        #self.count -= 1
        # publish the feedback solution_reauired False
        #while not rospy.is_shutdown():

        #    self.count += 1
        #    if (self.count >= 1):
        #        break
        
            rospy.loginfo("This state select the action required. It could be resume.")
            rospy.loginfo("self.current_selection is : %s", self.current_selection )
        

            if (self.current_selection != ""):   #Task already selected
                rospy.loginfo("further try")   
                userdata.action_selected = self.current_selection
                if (self.current_selection  == "MOVE"):
                    return 'goto_move'
                if (self.current_selection  == "DETECT"):
                    return 'goto_detect'        
                if (self.current_selection  == "GRASP"):
                    return 'goto_grasp'
            else:
                rospy.loginfo("first try")
                if (userdata.action_required == "move"):
                    userdata.action_selected = "MOVE"
                    self.current_selection = "MOVE"
                    return 'goto_move'
                if (userdata.action_required == "detect"):
                    userdata.action_selected = "DETECT"
                    self.current_selection = "DETECT"
                    return 'goto_detect'
                if (userdata.action_required == "grasp"):
                    userdata.action_selected = "GRASP"
                    self.current_selection = "GRASP"
                    return 'goto_grasp'
                else:
                    userdata.action_selected = "UNKNOWN"
                    self.current_selection = ""
                    return 'nok'
            

                    
#------------------- MOVE position section -------------------#
# Description : This smach state is to move the robot's base.
#                Move from position A to position B
# input keys : 'new_pos' is the B position. This target position is
#                a String which contains a word script server parameter
#                (e.g: "home", "kitchen" ...) or a list (e.g : "[-2.1, 1.05, 0.98]")
# output keys : None
# outcomes : 'ok' is returned if the new position is reached.
#            'nok' go to the error manager if there is an obstacle or an other error.


## Here is the intermediate move aimed at overcomming the problem encountered in the first move
class MOVE_E(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['ok','nok'],
                             input_keys=['intermediate_pos'])
   
        global hdl_torso
        global hdl_tray
        global hdl_arm
        global hdl_sdh
        global hdl_head
        global hdl_base
        #self.sc = script()
        #self.sc.Start()
        #self.sss = self.sc.sss
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.count = 0

    def execute(self, userdata):
        # publish the feedback current_state to the decision making
        while not rospy.is_shutdown():
            self.pub_fb.publish("Error correction MOVE begins ...")
            rospy.loginfo("New target is :%s", userdata.intermediate_pos) 
        #   rospy.sleep(1.0)
        #   self.count += 1
        #    if (self.count >= 1):
        #        break
        
             
            rospy.loginfo("Robot is trying to reach the intermediate  position : <<%s>>",userdata.intermediate_pos)
       
        #   for faster testing only #################################################### remove for the real tests #################
         #   if self.count < 2: 
         #       rospy.sleep(1.0)
         #       return 'nok'
         #   else:
         #       rospy.sleep(1.0)
         #       return 'ok'

            self.current_goal=userdata.intermediate_pos
            
            if (self.current_goal.find("[") == -1):   #this means we have received string with map coordinates not a predefined point,e.g. kitchen
                try:
                    hdl_base = sss.move("base",self.current_goal,blocking=False)
                                
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to send sss move, error: %s", error_message)
            else:
                self.tmppos = ""
                self.tmppos = userdata.self.current_goal.replace('[','')
                self.tmppos = userdata.self.current_goal.replace(']','')
                self.tmppos = userdata.self.current_goal.replace(',','')
                #print self.tmppos
                self.listtmp = self.tmppos.split()
                #print self.listtmp
                self.listtmp[0] = self.listtmp[0].replace('[','')
                self.listtmp[2] = self.listtmp[2].replace(']','')
                self.list = []            
                self.list.insert(0, float(self.listtmp[0]))
                self.list.insert(1, float(self.listtmp[1]))
                self.list.insert(2, float(self.listtmp[2]))
                print self.list
                try:
                    hdl_base = sss.move("base",self.list, blocking=False)
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to send move command via sss, error: %s", error_message)

 
            # waiting for base to reach the target position
            timeout = 0
            while True :
                try:
                    if (hdl_base.get_state() == 3):   #succeeded - final position reached return success to the user
                      return 'ok'
                    elif (hdl_base.get_state() == 2 or hdl_base.get_state() == 4):  #error or paused
                      return 'nok'
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to check hdl_base state, error: %s", error_message)
                rospy.sleep(0.5)
            
                print "3a"     
                # check if service is available
                service_full_name = '/base_controller/is_moving'
                try:
                  rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
                except rospy.ROSException, e:
                  error_message = "%s"%e
                  rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
                  return 'nok'
          
                print "4a"                
                # check if service is callable
                try:
                  is_moving = rospy.ServiceProxy(service_full_name,Trigger)
                  resp = is_moving()
                except rospy.ServiceException, e:
                  error_message = "%s"%e
                  rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                  return 'nok'
    
                print "5a"    
               # evaluate sevice response
                if not resp.success.data: # robot stands still
                  if timeout > 7:
                    print "Timeout...." 
                    return 'nok'          
                            





class MOVE(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ok','nok'],
                             input_keys=['new_pos'])
   
        global hdl_torso
        global hdl_tray
        global hdl_arm
        global hdl_sdh
        global hdl_head
        global hdl_base
        #self.sc = script()
        #self.sc.Start()
        #self.sss = self.sc.sss
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.count = 0
        self.current_goal = ""

    def execute(self, userdata):
        # publish the feedback current_state to the decision making
       while not rospy.is_shutdown():
            self.pub_fb.publish("MOVE begins ...")
            rospy.sleep(1.0)
            self.count += 1
           # if (self.count == 1):
           #     break
            
            rospy.loginfo("Robot is trying to reach a new position : %s",userdata.new_pos)
           

            # shortcut for testing without the simulation - ###################### REMOVE FOR THE REAL TESTS ################
            #if self.count < 3: 
             #  rospy.sleep(1.0)
             #  return 'nok'
          #  else:
          #     rospy.sleep(1.0)
          #     return 'ok'
                
            
            
            
            self.current_goal=userdata.new_pos
              

        ######                                        ##################
        ##Here, it necessary to know if 'new_pos' contains a script server 
        ##parameter (e.g: kitchen) or if contains x,y,z coordinates
        #########                                        ##############
            if (self.current_goal.find("[") == -1):   #this means we have received string with map coordinates not a predefined point,e.g. kitchen
                try:
                    hdl_base = sss.move("base",self.current_goal,blocking=False)
                                
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to send sss move, error: %s", error_message)
            else:
                self.tmppos = ""
                self.tmppos = userdata.self.current_goal.replace('[','')
                self.tmppos = userdata.self.current_goal.replace(']','')
                self.tmppos = userdata.self.current_goal.replace(',','')
                #print self.tmppos
                self.listtmp = self.tmppos.split()
                #print self.listtmp
                self.listtmp[0] = self.listtmp[0].replace('[','')
                self.listtmp[2] = self.listtmp[2].replace(']','')
                self.list = []            
                self.list.insert(0, float(self.listtmp[0]))
                self.list.insert(1, float(self.listtmp[1]))
                self.list.insert(2, float(self.listtmp[2]))
                print self.list
                try:
                    hdl_base = sss.move("base",self.list, blocking=False)
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to send sss move, error: %s", error_message)
      
        
        #    rospy.loginfo("(before manual wait) State base is : %s",hdl_base.get_state())
       #     print "Send command to UI_PRI:" # here feedback to the UI in topic \inteface_cmd should be sent for user to select new task (the current task is terminated)
           #return 'ok'
            
         #   print "1"         
         #   if (hdl_base.get_state() == 1) :  #active
         #       self.count = 0
            # publish the feedback current_state to the decision making
         #   while not rospy.is_shutdown():
         #       self.pub_fb.publish("MOVE is active : base try to reach target position "+userdata.new_pos+ ":", self.count)
         #       rospy.sleep(1.0)
         #       self.count += 1
         #       if (self.count == 1):
         #           break
            
            #print "2"             
        # wait for base to reach target position
            timeout = 0
            while True :
                try:
                    if (hdl_base.get_state() == 3):   #succeeded
                      return 'ok'
                    elif (hdl_base.get_state() == 2 or hdl_base.get_state() == 4):  #error or paused
                      return 'nok'
                except rospy.ROSException, e:
                    error_message = "%s"%e
                    rospy.logerr("unable to check hdl_base state, error: %s", error_message)
                rospy.sleep(0.5)
            
                #print "3"     
                # check if service is available
                service_full_name = '/base_controller/is_moving'
                try:
                  rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
                except rospy.ROSException, e:
                  error_message = "%s"%e
                  rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
                  return 'nok'
          
                #print "4"                
                # check if service is callable
                try:
                  is_moving = rospy.ServiceProxy(service_full_name,Trigger)
                  resp = is_moving()
                except rospy.ServiceException, e:
                  error_message = "%s"%e
                  rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                  return 'nok'
    
                #print "5"    
               # evaluate sevice response
                if not resp.success.data: # robot stands still
                  if timeout > 7:
                    #sss.say(["I can not reach my target position because my path or target is blocked, I will abort."],False)
                    print "Timeout...." 
                    return 'nok'          
                            
                  #  try:
                  #      rospy.wait_for_service('base_controller/stop',10)
                  #      stop = rospy.ServiceProxy('base_controller/stop',Trigger)
                  #      resp = stop()
                  #  except rospy.ServiceException, e:
                  #      error_message = "%s"%e
                  #      rospy.logerr("calling <<%s>> service not successfully, error: %s",service_full_name, error_message)
                    
                  else:
                    print "waiting for ",timeout," seconds"
                    timeout = timeout + 1
                    rospy.sleep(1)
                else:
                  timeout = 0   
            

 
#------------------- GRASP section -------------------#
# Description : this smach state let the robot grasp an object
# input keys : 'target_grasp' is the object to grasp. 
#             This String contains a list of coordinates like: [-3.2, 0.3, 0.98].
#             Then, the object will be putted on the tray.
# output keys : None
# outcomes : 'ok' if the process succeeded
#            'nok' if an error occured
class GRASP(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ok',
                                             'nok'],
                             input_keys=['target_grasp'])
        global hdl_torso
        global hdl_tray
        global hdl_arm
        global hdl_sdh
        global hdl_head
        global hdl_base
        #self.sc = script()
        #self.sc.Start()
        #self.sss = self.sc.sss
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.count = 0

    def execute(self, userdata):  
        # publish the feedback current_state to the decision making    
        while not rospy.is_shutdown():
            self.pub_fb.publish("GRASP is running ...")
            rospy.sleep(1.0)
            self.count += 1
            if (self.count == 1):
                break
            
        rospy.loginfo("grasp is running")

        listener = tf.TransformListener(True, rospy.Duration(10.0))

        # move arm to pregrasp position
        hdl_arm = sss.move("arm", "pregrasp", False)
        hdl_sdh = sss.move("sdh", "cylopen", False)

        # wait for arm movements to finish
        hdl_arm.wait()
        hdl_sdh.wait()

        # calculate tranformations, object coordinates needed for coordination
        self.tmp_coord = ""
        # suppression of some characters
        self.tmp_coord = userdata.target_grasp.replace('[','')
        self.tmp_coord = userdata.target_grasp.replace(']','')
        self.tmp_coord = userdata.target_grasp.replace(',','')
        
        self.list_coord = self.tmp_coord.split()

        self.list_coord[0] = self.list_coord[0].replace('[','')
        self.list_coord[2] = self.list_coord[2].replace(']','')
        print self.list_coord
        #[-3.2, 0.3, 0.98]
        
        object = PointStamped()
        object.header.stamp = rospy.Time.now()
        object.header.frame_id = "/map"
        object.point.x = float(self.list_coord[0])#-2.9
        object.point.y = float(self.list_coord[1])#0.05
        object.point.z = float(self.list_coord[2])#0.98
        sss.sleep(2)

        #if not self.sss.parse:
        object = listener.transformPoint('/arm_7_link', object)

        # grasp object
        hdl_arm = sss.move("arm",'pregrasp_2', False)
        sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
        hdl_sdh = sss.move("sdh", "cylclosed", False)

        # move object
        sss.move_cart_rel("arm",[[0.0, 0.4, 0.0], [0, 0, 0]])
        
        # put object on tray
        handle01 = sss.move("arm","grasp-to-tablet",False)
        sss.move("tray","up")

        #wait for arm movement to be finished
        handle01.wait()
        sss.move("sdh","cylopen")

        while True :
            if (hdl_arm.get_state() == 1) or (hdl_sdh.get_state() == 1):
                rospy.loginfog("ARM and/or SDH are still ACTIVE")
                self.count = 0
                # publish the feedback current_state to the decision making
                while not rospy.is_shutdown():
                   self.pub_fb.publish("GRASP active ...")
                   rospy.sleep(1.0)
                   self.count += 1
                   if (self.count == 2):
                       break
            else:
                if (hdl_arm.get_state() == 3):   #succeeded
                    rospy.loginfo("state arm is : %s",hdl_arm.get_state())
                    if (hdl_sdh == 2): #SDH succeeded to close, so grasp failed
                        return 'nok'
                    else:
                        return 'ok'
                else:
                    rospy.loginfo("state arm is : %s",hdl_arm.get_state())
                    return 'nok'


#------------------- DETECT section -------------------#
# Description : This smach state is to detect an object
#                in the workspace.
# input keys : 'target_detect' is the target
# output keys : None
# outcomes : 'ok' if the process succeeded
#            'nok' if an error occured      
class DETECT(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['ok',
                                             'nok'],
                             input_keys=['target_detect'])
        global hdl_torso
        global hdl_tray
        global hdl_arm
        global hdl_sdh
        global hdl_head
        global hdl_base
        #self.sc = script()
        #self.sc.Start()
        #self.sss = self.sc.sss
        self.pub_fb = rospy.Publisher('fb_executing_state', String)
        self.count = 0

    def execute(self, userdata):
        # publish the feedback current_state to the decision making
        while not rospy.is_shutdown():
            self.pub_fb.publish("DETECT is running ...")
            rospy.sleep(1.0)
            self.count += 1
            if (self.count == 2):
                break
        
        rospy.loginfo("detect is running")
        
        sss.detect(userdata.target_detect, False)
        
        
        if (hdl_detect.get_state() == 3) :  #success
            rospy.loginfo("Pose of %s are : %s",userdata.target_detect,sss.get_object_pose(userdata.target_detect))
            return 'ok'
        else:
            return 'nok'
        
#------------------- Error manager section -------------------#
# Description : This smach state is to detect an object
#                in the workspace.
# input keys : 'target_detect' is the target
# output keys : None
# outcomes : 'ok' if the process succeeded
#            'nok' if an error occured         

class wait_solution(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['try','not_try'],
                                input_keys=['current_action'],
                                output_keys=['solution_from_DM'])
        self.pub_fb = rospy.Publisher('fb_executing_solution', Bool)
        self.pub_fb2 = rospy.Publisher('fb_executing_state', String)
        self.count = 0
        
    def execute(self,userdata):
        while not rospy.is_shutdown():
            self.pub_fb.publish(True)
            rospy.sleep(1.0)
            self.count += 1
            self.pub_fb2.publish("Wait solution")

        
            rospy.loginfo("State : wait_solution")

            print userdata.current_action
        
            if userdata.current_action == 'INIT_COMPONENTS':
                current_state= 'Initial failed'
                exceptional_case_id=1
        
            if userdata.current_action == 'MOVE':
                current_state= 'Target not reached'
                exceptional_case_id=2
        
            if userdata.current_action == 'GRASP':
                current_state= 'Grasp failed'
                exceptional_case_id=3
        
            if userdata.current_action == 'DETECT':
                current_state= 'Object is not detected'
                exceptional_case_id=4
           
            rospy.wait_for_service('message_errors')
        
            print current_state
            
             
            s = xsrv.errorsResponse()
            
            try:
                message_errors = rospy.ServiceProxy('message_errors',xsrv.errors)
               
                s = message_errors(current_state, exceptional_case_id)
                print s.solution
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                
            

                self.pub_fb.publish(False)   
            
            print (s)
             
            if  (s.giveup == 1):
                return 'not_try'
            else:
                userdata.solution_from_DM = s.solution.__str__()#"home"#[1.0, 3.0, 0.0]"
                rospy.loginfo("New target is :%s", s.solution.__str__())    
                return 'try'
        
class SUCCESS(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete'],
                             output_keys=['act_result'])
   
    def execute(self, userdata):
        userdata.act_result = 3 # successful
        return 'complete'
        
class GIVENUP(smach.State):
    
    def __init__(self):
        smach.State.__init__(self, outcomes=['complete'],
                             output_keys=['act_result'])
   
    def execute(self, userdata):
        userdata.act_result = 4 # failed or given up
        return 'complete'
