#!/usr/bin/python
import roslib
roslib.load_manifest('srs_human_sensing')


import rospy


from sensor_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *

from cob_people_detection.srv import  *
from cob_people_detection_msgs.msg import *



def peopledetect(req):
        p=PeopleDetection()
        p2=PeopleDetection()

        pa=PeopleDetectionArray()
        p_srv=DetectPeopleResponse()
        pose=PoseStamped()
        pose.pose.position.x=1
        pose.pose.position.y=2
        pose.pose.position.z=3
        
        pose2=PoseStamped()
        pose2.pose.position.x=4
        pose2.pose.position.y=5
        pose2.pose.position.z=6
        
    
        p.pose=pose 
        p2.pose=pose2

        rospy.loginfo("kam was an")

      #  pa.detections.append(p)
      #  pa.header='leer'
        p_srv.people_list.detections.append(p)
        p_srv.people_list.detections.append(p2)

        
        return p_srv  


def people_detection_fake_server():
    rospy.init_node('detect_people_fake')
   
    s = rospy.Service('/cob_people_detection/detect_people', DetectPeople, peopledetect)
    rospy.loginfo("people_detection_fake_server ready.")
    rospy.spin()

if __name__ == "__main__":

    people_detection_fake_server()