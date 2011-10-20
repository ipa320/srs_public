#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_semantics_db')

import sys

import rospy
from srs_semantics_db.srv import *
from srs_semantics_db.msg import *
def test_semantics_db_insert_object():
    rospy.wait_for_service('insert_object_in_ontology')
    try:
        insert_object = rospy.ServiceProxy('insert_object_in_ontology', InsertObject)
        
        input1 = srs_semantics_db.msg.ABoxObject()
        input2 = srs_semantics_db.msg.TBoxObject()
        input1.object_id = 1
        input1.name = 'object'
        input2.class_id = 1
        input2.name = 'class'
        input2.description = 'class description'
        ret = insert_object(input1, input2)
        return ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    test_semantics_db_insert_object()
