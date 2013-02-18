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

sss = simple_script_server()


def main():
    
    rospy.init_node('prepare_robot_for_nav_test_node')
    rospy.sleep(1)
    
    sss.init('head', True)
    sss.init('tray', True)
    sss.init('arm', True)
    sss.init('torso', True)
    
    rospy.loginfo('Moving camera...')
    sss.move('head', 'front', True)
    
    rospy.loginfo('Moving tray...')
    sss.move('tray', 'down', True)
    
    rospy.loginfo('Moving arm...')
    sss.move('arm', 'folded', True)
    
    rospy.loginfo('Moving torso...')
    sss.move('torso', 'front_extreme', True)
    
    rospy.loginfo('Ready!')
    
    rospy.loginfo('Ready!')


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass