#!/usr/bin/env python
import roslib
roslib.load_manifest('srs_decision_making_experimental')
from simple_script_server import *
sss = simple_script_server()

rospy.init_node('test')
handle=sss.move('base','kitchen', False, 'linear')
rospy.sleep(5)
handle.set_failed(4)

