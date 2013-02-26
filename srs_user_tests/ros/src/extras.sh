#!/bin/sh
nohup xterm -bg OldLace -cr PowderBlue -fg NavyBlue -C -fa Verdana -sl 1 -sb -geometry 50x12 -hold -e "python $1/trigger_bag_client.py" &
nohup xterm -bg OldLace -cr PowderBlue -fg NavyBlue -C -fa Verdana -fs 72 -sl 1 -sb -geometry 9x1 -e "python $1/time_display.py" &
nohup rosrun rviz rviz -d `rospack find srs_user_tests`/ros/config/minimal.vcg &
