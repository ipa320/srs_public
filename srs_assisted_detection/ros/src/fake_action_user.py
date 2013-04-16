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
    s.data='succeeded'   #succeeded, give_up or retry
    try:
        user_intervention_on_detection = rospy.ServiceProxy('assisted_answer', UiAnswer)
        rospy.loginfo("this is fake_action_user")
        #a=array('i',[2,3,4,5])
        #resp1 = user_intervention_on_detection(a,0,s)   # detected object 0 has been succeeded
        
        resp1 = user_intervention_on_detection(0,s)   # detected object 0 has been succeeded
        
        #resp1 = user_intervention_on_detection(a,7,s)   # detected object 7 OUT OF INDEX
        rospy.loginfo(resp1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
if __name__ == "__main__":
    user_msg()
