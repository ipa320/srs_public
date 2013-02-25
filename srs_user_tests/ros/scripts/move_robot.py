#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_user_tests')
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose


def main():
    
    rospy.init_node('move_robot_to_given_place')
    
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped,latch=True)
    rospy.sleep(3)

    rospy.loginfo("Please move robot to start pose and press <Enter>.")
    
    pose = Pose()
    
    pose.position.x = rospy.get_param('~position_x')
    pose.position.y = rospy.get_param('~position_y')
    pose.position.z = rospy.get_param('~position_z')
    
    pose.orientation.x = rospy.get_param('~orientation_x')
    pose.orientation.y = rospy.get_param('~orientation_y')
    pose.orientation.z = rospy.get_param('~orientation_z')
    pose.orientation.w = rospy.get_param('~orientation_w')
    
    loc = PoseWithCovarianceStamped()

    loc.pose.pose = pose
    loc.header.frame_id = "/map"
    pub.publish(loc)
    rospy.sleep(1)
    pub.publish(loc)
    rospy.sleep(1)
    pub.publish(loc)


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
