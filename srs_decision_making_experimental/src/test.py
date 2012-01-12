#!/usr/bin/env python
import roslib
roslib.load_manifest('srs_decision_making_experimental')
from simple_script_server import *
sss = simple_script_server()
import tf
from geometry_msgs.msg import *
import time

rospy.init_node('test')

listener = tf.TransformListener()

rospy.sleep(5)

try:
    (trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
except rospy.ROSException, e:
    print "Transformation not possible: %s"%e

rb_pose = Pose2D()
rb_pose.x = trans[0]
rb_pose.y = trans[1]
rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
rb_pose.theta = rb_pose_rpy[2]
print rb_pose



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