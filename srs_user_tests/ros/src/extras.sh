#!/bin/sh
nohup xterm -bg OldLace -cr PowderBlue -fg NavyBlue -C -fa Verdana -sl 0 -sb -geometry 50x12 -e "python $1/trigger_bag_client.py" &
nohup xterm -bg OldLace -cr PowderBlue -fg NavyBlue -C -fa Verdana -fs 72 -sl 0 +sb -geometry 6x2 -e "python $1/time_display.py" &
nohup rosrun rviz rviz -d `rospack find srs_user_tests`/ros/config/minimal.vcg &
