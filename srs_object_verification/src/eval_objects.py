#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_object_verification');
import rospy

#from srs_semantics_db.srv import *
#from srs_semantics_db.msg import *
#from gdatabase.srv import *
from cob_3d_mapping_msgs.srv import *
from std_msgs.msg import *
from math import sqrt


class EvalObjects:
  def semantics_db_list_objects(self, class_id):
    rospy.wait_for_service('list_all_instances_of_class')
    try:
      list_objects = rospy.ServiceProxy('list_all_instances_of_class', ListInstancesOfClass)
      req = srs_semantics_db.msg.TBoxObject()
      req.class_id = class_id
      res = list_objects(req)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    object_poses = []
    rospy.wait_for_service('get_info_object')
    try:
      get_object_info = rospy.ServiceProxy('get_info_object', GetInfoObject)
      for l in res.list:
        object_poses.append(get_object_info(l.object_id).objectPose)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e
    return object_poses

  def semantics_db_get_object_info(self, object_id):
    rospy.wait_for_service('get_info_object')
    try:
      get_object_info = rospy.ServiceProxy('get_info_object', GetInfoObject)
      return get_object_info(object_id)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def map_list_objects(self, class_id):
    rospy.wait_for_service('get_objects_of_class')
    try:
      get_objects = rospy.ServiceProxy('get_objects_of_class', GetObjectsOfClass)
      req = UInt32()
      req.data = class_id
      res = get_objects(req)
      return res
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def verify_table(self, object_pose, table_list_map):
    d_min = 10000
    d_th = 0.5
    for t in table_list_map.objects.shapes:
      #pose_map = t.params[4:]
      d = sqrt((t.centroid.x-object_pose.position.x)**2+(t.centroid.y-object_pose.position.y)**2)
      if d < d_min and d < d_th:
        d_min = d
        closest_table = t
    if d_min < 10000:
      return closest_table
    else:
      return False


if __name__ == "__main__":
  class_id =1 #tables
  eo = EvalObjects()
  object_poses = eo.semantics_db_list_objects(class_id)
  for i in range(0,2):
    object_to_search = object_poses[i]
    print "Searching for object at " + str(object_to_search.position.x) + ", " + str(object_to_search.position.y)
    object_list_map = eo.map_list_objects(class_id)
    if class_id == 1:
      closest_table = eo.verify_table(object_to_search, object_list_map)
      if closest_table:
        print "table " + str(object_to_search.position.x) + "," + str(object_to_search.position.y) + " found at " + str(closest_table.params[4]) + "," + str(closest_table.params[5])
      else:
        print "table " + str(object_to_search.position.x) + "," + str(object_to_search.position.y) + " not found"
