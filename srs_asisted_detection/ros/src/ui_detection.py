#!/usr/bin/python
import roslib
roslib.load_manifest('srs_asisted_detection')


import rospy

from srs_asisted_detection.srv import *
from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from sensor_msgs.msg import *
from std_msgs.msg import *


detector_response=UiDetectorResponse()

def detectObjectSrv(req):
        rospy.loginfo("kam was an")
        rospy.wait_for_service('/object_detection/detect_object')
        try:
            string=String()
            string.data=req.object_name.data
            rospy.loginfo(string)

            srv_get_Objects = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
            roi=RegionOfInterest()
            resp1=srv_get_Objects(req.object_name,roi)
            #rospy.loginfo(resp1.object_list.detections[0])
           

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        global detector_response
        if len(resp1.object_list.detections) > 0:
            detector_response.object_list.header=resp1.object_list.header
            for x in range(len(resp1.object_list.detections)):
                detector_response.object_list.detections.insert(x,resp1.object_list.detections[x])
        
        s.shutdown()
        return detector_response  


def asisted_Detection_server():
 #   rospy.init_node('asisted_Detection_server')
    global s
    s = rospy.Service('asisted_detection', UiDetector, detectObjectSrv)
    rospy.loginfo("Asisted Detection ready.")
    s.spin()
    return detector_response

if __name__ == "__main__":

    asisted_Detection_server()