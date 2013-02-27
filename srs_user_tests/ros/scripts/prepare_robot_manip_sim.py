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
from simple_script_server import *
from std_msgs.msg import Empty as EmptyMsg

sss = simple_script_server()


def main():
    
    rospy.init_node('prepare_robot_for_manip_test_node')
    rospy.sleep(2)
    
    sim = rospy.get_param('/use_sim_time')
    
    torso_pos = rospy.get_param('~torso','home')
    
    if sim is True:
    
      rospy.loginfo('Waiting until end of spawning...')
      rospy.wait_for_message('/sim_spawned',EmptyMsg)
      rospy.sleep(5)
    
    sss.init('head', True)
    sss.init('tray', True)
    sss.init('arm', True)
    sss.init('torso', True)
    
    rospy.loginfo('Moving tray...')
    sss.move('tray', 'up', False)
    
    rospy.loginfo('Moving camera...')
    sss.move('head', 'back', False)
    
    
    rospy.loginfo('Moving arm...')
    #sss.move('arm', 'folded', True)
    sss.move('arm', 'look_at_table', True)
    
    
    rospy.loginfo('Moving torso...')
    sss.move('torso', torso_pos, True)
    #sss.move('torso', 'home', True)
    
    if sim is True:
    
      rospy.loginfo('Running in simulation, publishing to /sim_robot_init topic')
      
      pub = rospy.Publisher('/sim_robot_init', EmptyMsg,latch=True)
      pub.publish(EmptyMsg())
      pub.publish(EmptyMsg())
      pub.publish(EmptyMsg())
      
      rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
