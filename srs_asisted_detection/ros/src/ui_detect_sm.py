#!/usr/bin/python
import roslib
roslib.load_manifest('srs_asisted_detection')

import rospy

import threading

import smach
from smach import StateMachine ,Concurrence
from smach_ros import IntrospectionServer, SimpleActionState, MonitorState, IntrospectionServer

import ui_detection
import ui_answer

from geometry_msgs.msg import *

class UI_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1'],
                             output_keys=['detections'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state UI_detection')
        userdata.detections=ui_detection.asisted_Detection_server()
        return 'outcome1'

class Answer(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['success','retry'],
                             input_keys=['detections'],
                             output_keys=['action','pose','pose2d'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Answer')

        x=ui_answer.asisted_answer_server(userdata.detections)
        #action
        if x[0]==1:
            userdata.action=x[2]
            userdata.pose=x[1]
            return 'success'
        #bbmove    
        if x[0]==2:
            userdata.pose2d=x[2]
            return 'retry'
  
def main():
    rospy.init_node('sm_detect')

    sm0 = StateMachine(outcomes=['succeeded','not successful'])
    
    sm0.userdata.action=''
    sm0.userdata.pose=Pose()
    
    with sm0:
        smach.StateMachine.add('UI_detection', UI_detection(),
                                      transitions={'outcome1':'Answer'})
        smach.StateMachine.add('Answer', Answer(), 
                               transitions={'success':'succeeded','retry':'not successful'})
                     

    # Attach a SMACH introspection server
    sis = IntrospectionServer('smach_usecase_01', sm0, '/USE_CASE')
    sis.start()

    # Set preempt handler
    #smach.set_preempt_handler(sm0)

    # Execute SMACH tree in a separate thread so that we can ctrl-c the script
    smach_thread = threading.Thread(target = sm0.execute)
    smach_thread.start()

    # Signal handler
    rospy.spin()

if __name__ == '__main__':
    main()