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
 
    start_pose = Pose()
    
    start_pose.position.x = -3.0;
    start_pose.position.y = -0.2;
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
        
    try:    
        move_milk = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        move_milk(modelstate)  
    except rospy.ServiceException, e:
        error_message = "%s"%e
        rospy.logerr("have you put the milk into the simulation?")
        
        #setmodelstate = gazebo
        
        #setmodelstate.request.model_state = modelstate
        
      
       
    handle_tray = sss.move("tray", "down", False)  
    handle_tray.wait()
    
    _goal=xmsg.ExecutionGoal()
    _goal.action="move"
    _goal.parameter="[0.5 -1.6 1.57]"
    _goal.priority=0
    client.send_goal(_goal)  
    client.wait_for_result()
    sss.say(["back to charging station, I am ready for new tasks"],False)
    
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
