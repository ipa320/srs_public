#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_object_verification');
import rospy

from srs_semantics_db.srv import *
from srs_semantics_db.msg import *
from gdatabase.srv import *
from geometry_msgs.msg import *

def list_service_cb(req):
  if req.tbox_class.class_id != 1:
    print "Object class not supported"
    return False
  table1 = ABoxObject()
  table1.object_id = 0
  table2 = ABoxObject()
  table2.object_id = 1
  res = ListInstancesOfClassResponse()
  res.list = [table1,table2]
  return res
  
def info_service_cb(req):
  res = GetInfoObjectResponse()
  if req.objectID == 0:
    res.objectName = "table0"
    res.objectPose = Pose(position=Point(0,0,0), orientation=Quaternion(0,0,0,0))
    res.classID = 1
  elif req.objectID == 1:
    res.objectName = "table1"
    res.objectPose = Pose(position=Point(10,10,0), orientation=Quaternion(0,0,0,0))
    res.classID = 1 
  else:
    print "Object info not found"
    return False
  return res

def start_server():
    rospy.init_node('semantic_db_test_server')
    s = rospy.Service('list_all_instances_of_class', ListInstancesOfClass, list_service_cb)
    s = rospy.Service('get_info_object', GetInfoObject, info_service_cb)
    print "Ready to return objects from db."
    rospy.spin()

if __name__ == "__main__":
    start_server()