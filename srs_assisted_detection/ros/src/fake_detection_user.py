#!/usr/bin/python
import roslib
roslib.load_manifest('srs_assisted_detection')


import rospy

from cob_object_detection_msgs.msg import *
from srs_assisted_detection.srv import *
from std_msgs.msg import *



def user_msg():    
    rospy.wait_for_service('assisted_detection')
    string=String()
    string.data='junk'   #this is useless, it will be overwritten by the input key of the detection state
    try:
        add_two_ints = rospy.ServiceProxy('assisted_detection', UiDetector)
        rospy.loginfo("client")
        resp1 = add_two_ints(string)
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
if __name__ == "__main__":
    user_msg()
