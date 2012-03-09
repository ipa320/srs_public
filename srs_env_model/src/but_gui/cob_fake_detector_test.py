#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_env_model');
import rospy
from math import fabs
import sys

from cob_object_detection_msgs.srv import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import RegionOfInterest
from srs_env_model.srv import AddObjectWithBoundingBox, RemovePrimitive


def detect_object(object_name):
    print "Waiting for detector... ",
    sys.stdout.flush()
    rospy.wait_for_service('object_detection/detect_object')
    print 'ok'
    
    
    print "Detecting milk...",
    sys.stdout.flush() 
    detect_object = rospy.ServiceProxy('object_detection/detect_object', DetectObjects)
    try:
        object_name = String()
        object_name.data = 'milk'
        roi = RegionOfInterest()
        
        objects = detect_object(object_name, roi)
    except:
        print "ERROR!\nexiting..."
        exit()
    
    print "detected" 
    
    return objects

def add_object_to_scene(object):
    print object
    
    print "Waiting for service... ",
    sys.stdout.flush()
    rospy.wait_for_service('but_gui/add_object')
    print "ok"
    
    print "Adding object to scene...",
    sys.stdout.flush()
    add_object = rospy.ServiceProxy('but_gui/add_object_with_bounding_box', AddObjectWithBoundingBox)
    try:
        color = ColorRGBA()
        color.r = 1
        color.g = 1
        color.a = 1
        pose = object.pose.pose
        
        add_object(object.pose.header.frame_id, object.label, "Detected milk", pose, object.bounding_box_min, object.bounding_box_max, color, None, 'package://cob_gazebo_worlds/Media/models/milk_box.dae', True)
    except:
        print "ERROR!\nexiting..."
        exit()
        
    print "done"
 
  
if __name__ == '__main__':
    
    # remove old objects from scene
    rospy.wait_for_service('but_gui/remove_primitive')
    remove_object = rospy.ServiceProxy('but_gui/remove_primitive', RemovePrimitive)
    try:
        resp = remove_object("milk")
    except:
        pass
    
    objects = detect_object("milk")
    
    add_object_to_scene(objects.object_list.detections[0])
    
    
