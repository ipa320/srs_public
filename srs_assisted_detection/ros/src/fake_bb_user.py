#!/usr/bin/python
import roslib
roslib.load_manifest('srs_assisted_detection')

from array import *
import rospy

from cob_object_detection_msgs.msg import *
from srs_assisted_detection.srv import *
from std_msgs.msg import *

from geometry_msgs.msg import *

def user_msg():    
    rospy.wait_for_service('assisted_BBmove')

    try:
        add_two_ints = rospy.ServiceProxy('assisted_BBmove', BBMove)
        rospy.loginfo("client")
        pose=Pose()
        pose.position.x=0
        pose.position.y=0
        pose.position.z=0

        resp1 = add_two_ints(2,4,4,pose)
        rospy.loginfo(resp1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
if __name__ == "__main__":
    user_msg()
