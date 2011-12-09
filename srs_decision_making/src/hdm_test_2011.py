#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the SRS DM action, including the
# goal message and the result message.
import srs_decision_making.msg as xmsg
# include script server, to move the robot
from simple_script_server import simple_script_server
sss = simple_script_server()

import geometry_msgs.msg as geomery
import gazebo.msg as gazebo
from gazebo.srv import SetModelState
# msg imports
from geometry_msgs.msg import *

from cob_object_detection_msgs.srv import *
from cob_object_detection_msgs.msg import *
from gazebo.srv import *
import gazebo.msg as gazebo


#! /usr/bin/env python

import roslib; roslib.load_manifest('srs_decision_making')
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the SRS DM action, including the
# goal message and the result message.
import srs_decision_making.msg as xmsg

def DM_client():
    # Creates the SimpleActionClient, passing the type of the action
    # constructor.
    client = actionlib.SimpleActionClient('srs_decision_making_actions', xmsg.ExecutionAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    
    # creates action sequence for the action server
    
    # Creates a goal to send to the action server.
    _goal=xmsg.ExecutionGoal()
    _goal.action="move"
    _goal.parameter="[0.5 -1.6 1.57]"
    _goal.priority=0
    client.send_goal(_goal)  
    client.wait_for_result()
    print("ready to start")
    sss.wait_for_input()

 
    _goal.action="get"# msg imports
    _goal.parameter="milk"
    _goal.priority=0
    client.send_goal(_goal)  
    client.wait_for_result()
    print("milk ready")
    sss.wait_for_input()


    _goal.action="move"
    _goal.parameter="[1.47 -0.7 0.75]"
    _goal.priority=0
    client.send_goal(_goal)  
    client.wait_for_result()
    print("milk delivered")
    sss.wait_for_input()
    
    
    start_pose = Pose()
    
    start_pose.position.x = -3.15;
    start_pose.position.y = -0.1;
    start_pose.position.z = 1.02;
    start_pose.orientation.x = 0.0;
    start_pose.orientation.y = 0.0;
    start_pose.orientation.z = 0.0;
    start_pose.orientation.w = 0.0;
        
    start_twist = Twist()
    start_twist.linear.x = 0.0;
    start_twist.linear.y = 0.0;
    start_twist.linear.z = 0.0;
    start_twist.angular.x = 0.0;
    start_twist.angular.y = 0.0;
    start_twist.angular.z = 0.0;
        
    modelstate = gazebo.ModelState
    modelstate.model_name = "milk_box";
    modelstate.reference_frame = "world";
    modelstate.pose = start_pose;
    modelstate.twist = start_twist;
        
        
    move_milk = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        
        #setmodelstate = gazebo
        
        #setmodelstate.request.model_state = modelstate
        
    move_milk(modelstate)    
       
    print("confirm milk has been taken")
    sss.wait_for_input()
    client.wait_for_result()
    handle_tray = sss.move("tray", "down", False)  
    handle_tray.wait()
    
    _goal.action="move"
    _goal.parameter="[1 -1.6 1.57]"
    _goal.priority=0
    client.send_goal(_goal)  
    client.wait_for_result()
    print("back to charge")
    
    return client.get_result()





if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('dm_client1')
        result = DM_client()
        rospy.loginfo('result %s',result)
        # print ("Result:" result)
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
