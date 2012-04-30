#!/usr/bin/python
import roslib
roslib.load_manifest('srs_assisted_detection')


import rospy

from cob_object_detection_msgs.msg import *
from srs_assisted_detection.srv import *
from std_msgs.msg import *

from geometry_msgs.msg import *

from array import array


def user_msg():    
    rospy.wait_for_service('assisted_answer')
    s=String()
    s.data='test'
    try:
        add_two_ints = rospy.ServiceProxy('assisted_answer', UiAnswer)
        rospy.loginfo("client")
        a=array('i',[2,3,4,5])
        resp1 = add_two_ints(a,0,s)
        rospy.loginfo(resp1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
if __name__ == "__main__":
    user_msg()
