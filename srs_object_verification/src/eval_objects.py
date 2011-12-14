#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_object_verification');
import rospy

from srs_semantics_db.srv import *
from srs_semantics_db.msg import *
from gdatabase.srv import *


def semantics_db_list_objects():
  rospy.wait_for_service('list_all_instances_of_class')
  try:
    list_objects = rospy.ServiceProxy('list_all_instances_of_class', ListInstancesOfClass)
    req = srs_semantics_db.msg.TBoxObject()
    req.class_id = 1
    req.name = 'table'
    req.description = 'all kinds of tables'
    res = list_objects(req)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  object_poses = []
  rospy.wait_for_service('GetInfoObject')
  try:
    get_object_info = rospy.ServiceProxy('GetInfoObject', GetInfoObject)
    for l in res.list:
      object_info.append(get_object_info(l.object_id).objectPose)
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
  return object_poses

    
def map_list_objects():
  rospy.wait_for_service('get_objects_of_class')
  try:
    list_objects = rospy.ServiceProxy('get_objects_of_class', GetObjectsOfClass)
    req = std_msgs.Int32()
    req.data = 1
    res = get_objects(req)
    return res
  except rospy.ServiceException, e:
    print "Service call failed: %s"%e
        
        
if __name__ == "__main__":
  object_poses = semantics_db_list_objects()
  table_list_map = map_list_objects()
  