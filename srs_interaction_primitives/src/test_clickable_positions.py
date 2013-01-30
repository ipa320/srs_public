#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by Robo@FIT group.
# 
# Author: Tomas Lokaj
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: 12/09/2012
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

import roslib; roslib.load_manifest('srs_interaction_primitives')
import rospy
import actionlib
from std_msgs.msg import *
from visualization_msgs.msg import  *
from geometry_msgs.msg import  *
from srs_interaction_primitives.msg import *
import random
import time

from srs_interaction_primitives.srv import ClickablePositions 


if __name__ == '__main__':
    rospy.init_node('clickable_positions_action_client', anonymous=True)
    
    #===========================================================================
    # rospy.wait_for_service('interaction_primitives/clickable_positions')
    # click_positions = rospy.ServiceProxy('interaction_primitives/clickable_positions', ClickablePositions)
    # 
    # color = ColorRGBA()
    # color.r = random.uniform(0, 1)
    # color.g = random.uniform(0, 1)
    # color.b = random.uniform(0, 1)
    # color.a = 1;    
    # 
    # radius = random.uniform(0, 1)
    # 
    # positions = []
    # for i in range(0, random.randint(2, 10)):
    #    positions.append(Point(random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0)))
    # 
    # frame_id = "/world"
    # 
    # topic = str(random.randint(0, 10000))
    # 
    # resp = click_positions(frame_id, topic, radius, color, positions)
    # 
    #===========================================================================
    
    client = actionlib.SimpleActionClient("clickable_positions_server", ClickablePositionsAction)
    client.wait_for_server()
    rospy.loginfo("Server ready")

    goal = ClickablePositionsGoal()
    color = ColorRGBA()
    color.r = random.uniform(0, 1)
    color.g = random.uniform(0, 1)
    color.b = random.uniform(0, 1)
    color.a = 1;    
    goal.topic_suffix = str(random.randint(0, 10000))
    goal.color = color
    goal.radius = random.uniform(0, 1)
    for i in range(0, random.randint(2, 10)):
        goal.positions.append(Point(random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0), random.uniform(-10.0, 10.0)))
    goal.frame_id = "/world"
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(50.0))
    if client.get_state() == 3:
        rospy.loginfo("Goal completed:")
        print client.get_result()
    else:
        rospy.logwarn("Action was preempted")
