#!/usr/bin/env python
import roslib
roslib.load_manifest('srs_decision_making')
from simple_script_server import *
sss = simple_script_server()
import tf
from geometry_msgs.msg import *
import time

from srs_knowledge.srv import *


rospy.init_node('test')

getWorkspace = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
all_workspaces_on_map = getWorkspace(os.environ['ROBOT_ENV'], True)

target_object_name ='Table0'
#rospy.INFO ('target object is:', target_object_name)
        
#get the index of the target workspace e.g. table0    
index_of_the_target_workspace = all_workspaces_on_map.objects.index(target_object_name)
#get the pose of the workspace from knowledge service
target_object_pose = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].pose
#get the houseHoldID of the workspace 
object_id = all_workspaces_on_map.houseHoldId[index_of_the_target_workspace]

rospy.loginfo ("target name: %s", target_object_name)      
rospy.loginfo ("target pose: %s", target_object_pose)
rospy.loginfo ("target id: %s", object_id)


"""

= 'base_link';
=self.listenr

tf.TransformListener.transformPose('/map', zero_pose_, robot_pose_global_)

robot_position_2D = Pose2D()

robot_position_2D = position.x
x_last_ = robot_pose_global_.pose.position.x;
y_last_ = robot_pose_global_.pose.position.y;
theta_last_ = tf::getYaw(robot_pose_global_.pose.orientation);


geometry_msgs::PoseStamped robot_pose_global_;

geometry_msgs::PoseStamped goal_pose_global_;
geometry_msgs::PoseStamped zero_pose_;

zero_pose_.pose.position.x = 0.0;
zero_pose_.pose.position.y = 0.0;
zero_pose_.pose.position.z = 0.0;
zero_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
zero_pose_.header.frame_id = robot_frame_;

try{
tf_listener_.transformPose('/map', zero_pose_, robot_pose_global_);
}
catch(tf::TransformException& ex){
ROS_WARN("Failed to find robot pose in global frame %s", 'map';
return zero_pose_;
}
"""