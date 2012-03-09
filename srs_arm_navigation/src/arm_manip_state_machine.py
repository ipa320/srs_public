#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_arm_navigation')
import rospy
#import smach
#import smach_ros
import actionlib

from srs_arm_navigation.msg import *

def main():
  rospy.init_node('arm_manip_action_test')
  rospy.loginfo("Node for testing actionlib server")
 
  client = actionlib.SimpleActionClient('manual_arm_manip_action',ManualArmManipAction)
  
  rospy.loginfo("Waiting for server...")
  client.wait_for_server()
  goal = ManualArmManipGoal()
  goal.away = True
  client.send_goal(goal)
  
  #timeout = 240.0
  rospy.loginfo("Waiting for result")
  client.wait_for_result()
  rospy.loginfo("I have result!! :-)")
  
  result = client.get_result()
  
  if result.success:
    rospy.loginfo("Success!")
    
  if result.failed:
    rospy.loginfo("Failed :-(")
    

  if result.timeout:
    rospy.loginfo("User is sleeping")
  
  rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())
  
if __name__ == '__main__':
  main()