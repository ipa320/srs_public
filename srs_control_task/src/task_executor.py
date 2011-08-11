#!/usr/bin/python
#################################################################################
# SRS project - task_executor.py                                     #
# Place : Manufacturing Engineering Centre, Cardiff University                  #
# Author : Damien LEBON, Sylvain Chardonneau & Gwendal LOUBES                   #
# Date : August 2011                                                            #
#-------------------------------------------------------------------------------#
# Description : This is the actionlib server called to execute a task.          #
# The client is the decision making. 
#################################################################################

import roslib; roslib.load_manifest('srs_control_task')
import rospy
import smach
import smach_ros

from actionlib import *
from actionlib.msg import *

import time
from string import *

import srs_control_task.msg as xmsg

# to use all the smach state
from high_level_script_server import *


_feedback = xmsg.ExecutionFeedback()
_result = xmsg.ExecutionResult()

class Executor():
    def __init__(self):
        # create the actionlib server with the messages of the action Execution.
        self._as = actionlib.SimpleActionServer(rospy.get_name(), xmsg.ExecutionAction, self.launch, False)  
        self.start()
  
    def start(self):
        self._as.start()
        rospy.loginfo("Waiting for wake up the server ...")

# create the main state machine named ACT.
# it contains INIT_COMPONENTS, RUNNING, FAILED
# input : action required and the parameter
# output : smach State Machine object
    def init_sm_act(self,actiongoal,paramgoal):        
        self.temp = smach.StateMachine(outcomes=['end'])

        self.temp.userdata.action = actiongoal
        self.temp.userdata.param_move = paramgoal
        self.temp.userdata.param_detect = paramgoal
        self.temp.userdata.param_grasp = paramgoal
        
        self.running = self.init_sm_running()
        
        self.fail = self.init_sm_failed()
        
        with self.temp:
            smach.StateMachine.add('INIT_COMPONENTS',INIT_COMPONENTS(),
                                  transitions={'trigger':'RUNNING',
                                               'init_error':'FAILED'},
                                  remapping={'action_req':'action'})
       
            smach.StateMachine.add('RUNNING',self.running,
                                   transitions={'success':'end',
                                                'fail':'FAILED'},
                                   remapping={'action_run':'action',
                                              'parameter_move':'param_move',
                                              'parameter_detect':'param_detect',
                                              'parameter_grasp':'param_grasp'})
            smach.StateMachine.add('FAILED',self.fail,
                                   transitions={'resume':'RUNNING'})
        
        #Your state machine needs to run an introspection server to allow the smach viewer to connect to it.   
        sis = smach_ros.IntrospectionServer('ACT_SM', self.temp, '/START')
        sis.start()
        
        return self.temp


# create the sub state machine named RUNNING.
# it contains SELECT, MOVE, DETECT, GRASP.
# input : None
# output : smach State Machine object      
    def init_sm_running(self):
        self.tmprun = smach.StateMachine(outcomes=['success','fail'],
                                input_keys=['action_run',
                                            'parameter_move',
                                            'parameter_detect',
                                            'parameter_grasp'])

        # Open the container
        with self.tmprun :
            # Add states to the container
            smach.StateMachine.add('SELECT',SELECT(),
                                   transitions={'goto_move':'MOVE',
                                                'goto_detect':'DETECT',
                                                'goto_grasp':'GRASP'},
                                   remapping={'action_required':'action_run'})
            smach.StateMachine.add('MOVE',MOVE(),
                                   transitions={'ok':'success',
                                                'nok':'fail'},
                                   remapping={'new_pos':'parameter_move'})
            smach.StateMachine.add('DETECT',DETECT(),
                                   transitions={'ok':'success',
                                                'nok':'fail'},
                                   remapping={'target_detect':'parameter_detect'})
            smach.StateMachine.add('GRASP',GRASP(),
                                   transitions={'ok':'success',
                                                'nok':'fail'},
                                   remapping={'target_grasp':'parameter_grasp'})
        return self.tmprun
    
# create the sub state machine named FAILED.
# it contains Wait_solution and MOVE_errors
# input : None
# output : smach State Machine object   
    def init_sm_failed(self):
        self.Failed = smach.StateMachine(outcomes=['resume'])
        
        #self.Failed.userdata.transfert = ""
        
        # Open the container
        with self.Failed :
                smach.StateMachine.add('wait_solution',wait_solution(),
                                   transitions={'try':'move_errors'},
                                   remapping={'useless':'transfert',
                                              'solution_from_DM':'transfert'})
            

                smach.StateMachine.add('move_errors',MOVE(),
                                       transitions={'ok':'resume',
                                                    'nok':'wait_solution'},
                                       remapping={'new_pos':'transfert'})
        
        return self.Failed

# This function is called when a rostopic message is published.
# Then, the action lib feedback is published to the decision making.
    def callback_fb_solution_req(self, data):
        #rospy.loginfo("I heard %s",data.data)
        _feedback.solution_required = data.data
        self._as.publish_feedback(_feedback)
        
    def callback_fb_current_state(self, data):
        #rospy.loginfo("I heard %s",data.data)
        _feedback.current_state = data.data
        self._as.publish_feedback(_feedback)

# This function is triggered when a goal is received.
# The state machine start here.
    def launch(self, goal):
        _feedback.current_state = "Launching the state machine"
        _feedback.solution_required = False
        _feedback.exceptional_case_id = 0
        self._as.publish_feedback(_feedback)
        
        rospy.Subscriber("fb_executing_solution", Bool, self.callback_fb_solution_req)
        rospy.Subscriber("fb_executing_state", String, self.callback_fb_current_state)
        
        rospy.loginfo('Current goal :\n%s',goal)
    
        self.action = goal.action
        self.param = goal.parameter
        rospy.loginfo('Current action %s',self.action)
        rospy.loginfo('Current parameter %s',self.param)
        
        self.act = self.init_sm_act(self.action, self.param)
        
        self._as.is_active()
        # Execute the state machine
        outcome = self.act.execute()
        
        self._as.is_active()
        
        _result.return_value = 2
        
        self._as.set_succeeded(_result)


if __name__ == "__main__":
    rospy.init_node('task_executor')
    Executor()
    
    rospy.spin()
