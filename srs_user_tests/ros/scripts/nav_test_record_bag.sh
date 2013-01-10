#!/bin/bash
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by Robo@FIT group.
#
# Author: Michal Spanel (spanel@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: 11/12/2012
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

# Description:
# Records topics required to evaluate results of the navigation user test.

# Default filename prefix
BAG_PREFIX="/tmp/nav_test_log"

# Topics to record
TOPICS="/spacenav/offset \
/spacenav/rot_offset \
/cob_interactive_teleop/feedback"

# Process script's arguments
OPT=-
for i in $@; do  
    if [ $OPT = - ]; then
        if [ $i = -p ]; then
            OPT=$i
#        elif [ $i = -t ]; then
#            OPT=$i
        fi
    else
        if [ $OPT = -p ]; then
            BAG_PREFIX=$i
#        elif [ $OPT = -t ]; then
#            START_TIME=$i
        fi
        OPT=-
    fi  
done

# Record all required topics
rosbag record $TOPICS --output-prefix=$BAG_PREFIX --buffsize=1024

