#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_human_sensing')
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from cob_people_detection_msgs.msg import PeopleDetectionArray
from srs_human_sensing.srv import Comp_HS_Detections
import tf
from std_msgs.msg import Header


def callback_LD(data):
    rospy.loginfo("I heard from LD:")
    leg_detections = data.points
    global legs 
    legs = []
    for i in range (len(leg_detections)):
       legs.append(leg_detections[i])
       print leg_detections[i].x,",",leg_detections[i].y
    
    match_detections()


def callback_FD(data):
    rospy.loginfo("I heard from FD:")
    face_detections = data.detections
    
    
    global faces 
    faces = []

    for i in range (len(face_detections)):
       
    

       try:

          detection1 = tflistener.transformPose('map', face_detections[i].pose)        
          newpoint = Point32(detection1.pose.position.x,detection1.pose.position.y,detection1.pose.position.z)
          faces.append(newpoint)

       except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
         print "tf transform error"
   
    match_detections () 

def match_detections ():
    distances = [] 
    global legs
    global faces
    global pub

    pc = PointCloud()
    pc.header = Header(0,rospy.get_rostime(),'/map')
   
     
    
    

    for m in range (len(faces)):
                pc.points.append(Point32(faces[m].x,faces[m].y,1.5))
   
    for l in range (len(legs)):
         pc.points.append(Point32(legs[l].x,legs[l].y,1.0))
         distancecol = []
         for m in range (len(faces)):
                
           # measure the distance between the detections and store them
                dist = (sqrt((faces[m].x-legs[l].x)**2+(faces[m].y-legs[l].y)**2))
                distancecol.append (dist)
                 
         distances.append (distancecol) 
         
    print "distances"
    print distances
    
     
    
    
   
    
    pub.publish (pc)
      
def handle_compare_hs_detections ():
    return True

def listener():
    
    rospy.Subscriber("leg_detections_cloud", PointCloud , callback_LD)
    rospy.Subscriber("/cob_people_detection/face_position_array", PeopleDetectionArray , callback_FD)
    rospy.spin()

if __name__ == '__main__':
    print "Listening ..."


    rospy.init_node('SRS_HS_listener', anonymous=True)
    legs = []
    faces = []
    tflistener = tf.TransformListener()
    tflistener.waitForTransform("/head_cam3d_link", "/map", rospy.Time(), rospy.Duration(5.0))
    pub = rospy.Publisher('people_detections_cloud', PointCloud)
    serv = rospy.Service ('compare_hs_detections',Comp_HS_Detections,handle_compare_hs_detections)
    
    listener()
