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

import roslib; roslib.load_manifest('srs_assisted_arm_navigation')
import rospy
import actionlib

from arm_navigation_msgs.msg import MoveArmAction
from arm_navigation_msgs.msg import MoveArmGoal
from arm_navigation_msgs.msg import PositionConstraint
from arm_navigation_msgs.msg import OrientationConstraint
from arm_navigation_msgs.msg import ArmNavigationErrorCodes

def main():
    
    rospy.init_node('move_arm_action_test')
    rospy.loginfo('Node for testing move_arm action')
    
    client = actionlib.SimpleActionClient('/move_arm',MoveArmAction)
    
    client.wait_for_server()
    
    rospy.loginfo('Server is available, let\'s start')
    
    goal = MoveArmGoal()
    
    goal.motion_plan_request.group_name = 'arm'
    goal.motion_plan_request.num_planning_attempts = 5
    goal.motion_plan_request.planner_id = ''
    goal.planner_service_name = '/ompl_planning/plan_kinematic_path'
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(15.0)
    
    # example of valid position
    # - Translation: [-0.623, -0.460, 1.162]
    # - Rotation: in Quaternion [0.739, -0.396, -0.533, 0.116]
    
    # default position
    # - Translation: [-0.316, -0.816, 1.593]
    # - Rotation: in Quaternion [0.380, 0.153, -0.656, 0.634]

    
    pos_const = PositionConstraint()
    
    pos_const.header.frame_id = 'base_link'
    pos_const.link_name = 'sdh_palm_link'
    #pos_const.link_name = 'arm_7_link'
    
    # default position
    #pos_const.position.x =  -0.316
    #pos_const.position.y =  -0.816
    #pos_const.position.z = 1.593
    pos_const.position.x =  -0.623
    pos_const.position.y =  -0.460
    pos_const.position.z = 1.162
    
    pos_const.constraint_region_shape.type = pos_const.constraint_region_shape.BOX
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    pos_const.constraint_region_shape.dimensions.append(2*0.02)
    
    pos_const.constraint_region_orientation.x = 0.0
    pos_const.constraint_region_orientation.y = 0.0
    pos_const.constraint_region_orientation.z = 0.0
    pos_const.constraint_region_orientation.w = 1.0
    
    pos_const.weight = 1.0
    
    #pos_const.target_point_offset.x = 0.02
    #pos_const.target_point_offset.y = 0.02
    #pos_const.target_point_offset.z = 0.02
    

    
    or_const = OrientationConstraint()
    
    or_const.header.frame_id = 'base_link'
    or_const.link_name = 'sdh_palm_link'
    #or_const.link_name = 'arm_7_link'
    
    #or_const.type = xxx
    
    or_const.weight = 1.0
    
    #or_const.orientation
    
    # default orientation
    #or_const.orientation.x = 0.380
    #or_const.orientation.y = 0.153
    #or_const.orientation.z = -0.656
    #or_const.orientation.w = 0.634
    or_const.orientation.x = 0.739
    or_const.orientation.y = -0.396
    or_const.orientation.z = -0.533
    or_const.orientation.w = 0.116
    
    or_const.absolute_pitch_tolerance = 0.04
    or_const.absolute_roll_tolerance = 0.04
    or_const.absolute_yaw_tolerance = 0.04
    
    # --------------------------------------------------
    
    goal.motion_plan_request.goal_constraints.orientation_constraints.append(or_const)
    goal.motion_plan_request.goal_constraints.position_constraints.append(pos_const)
    
    client.send_goal(goal)
    
    client.wait_for_result()
    
    res = client.get_result()
    
    #rospy.loginfo(client.get_state())
    
    print "Result"
    print res
    
    
    
if __name__ == '__main__':
  main()
