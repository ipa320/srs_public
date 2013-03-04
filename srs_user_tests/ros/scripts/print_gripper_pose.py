#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#
import roslib; roslib.load_manifest('srs_user_tests')
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Vector3
from visualization_msgs.msg import InteractiveMarkerUpdate
from threading import Lock
from tf import TransformListener

tf = None
gr_lock = Lock()
gr_pose = None
planning_started = False

def update_cb(msg):
    
    global planning_started
    global tf
    global gr_pose
    global gr_lock
    
    if len(msg.poses) != 1:
        
         return
    
    if msg.poses[0].name  != "MPR 0_end_control":
        
        return
    
    tmp = PoseStamped()
    tmp.header.stamp = rospy.Time(0)
    tmp.header.frame_id = msg.poses[0].header.frame_id
    tmp.pose = msg.poses[0].pose
    
    tf.waitForTransform('/map',msg.poses[0].header.frame_id,rospy.Time(0), rospy.Duration(1.0))
    tmp = tf.transformPose('/map',tmp)
    
    gr_lock.acquire()
    gr_pose = tmp.pose
    gr_lock.release()
    
    if (planning_started == False):
        
        rospy.loginfo('Planning has been started!')
        planning_started = True

def main():
    
    global planning_started
    global tf
    global gr_pose
    global gr_lock
    
    rospy.init_node('print_gripper_pose_node')
    
    tf = TransformListener()
    
    bb_pose = Pose()
    lwh = Vector3()
    
    bb_pose.position.x = rospy.get_param("~bb/position/x")
    bb_pose.position.y = rospy.get_param("~bb/position/y")
    bb_pose.position.z = rospy.get_param("~bb/position/z")
    
    lwh.x = rospy.get_param("~bb/lwh/x")
    lwh.y = rospy.get_param("~bb/lwh/y")
    lwh.z = rospy.get_param("~bb/lwh/z")
    
    rospy.loginfo('Please start arm planning')
    
    subs = rospy.Subscriber('/planning_scene_warehouse_viewer_controls/update',InteractiveMarkerUpdate,update_cb)
    
    r = rospy.Rate(0.5)
    
    while planning_started == False:
        
        r.sleep()      
    
    raw_input("Please press enter when finished!")
    
    subs.unregister()
    
    gr_lock.acquire()
    
    gr_pose.position.x -= bb_pose.position.x
    gr_pose.position.y -= bb_pose.position.y
    gr_pose.position.z -= bb_pose.position.z
    
    gr_lock.release()
    
    print  "gripper:"
    print " position:"
    print "  x: " + str(gr_pose.position.x)
    print "  y: " + str(gr_pose.position.y)
    print "  z: " + str(gr_pose.position.z)
    
    
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass