#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_semantics_db'); roslib.load_manifest('json_prolog')
import rospy
import json_prolog
from srs_semantics_db.srv import *
from srs_semantics_db.msg import *

class SRSSemanticsDB:

    def __init__(self):
        print ''
        rospy.init_node('test_json_prolog')
    
        self.prolog = json_prolog.Prolog()
        query = self.prolog.query("member(A, [1,2,3,4]), B = ['x', A]")
    
        for sol in query.solutions():
            print 'Found solution. A = %s, B = %s' %(sol['A'], sol['B'])
        query.finish()
        
        query = self.prolog.query("owl_subclass_of(A, knowrob:'FoodOrDrink').")
        for sol in query.solutions():
            print 'Found solution. A = %s' %(sol['A'])

        query.finish()
    
        self.insert_object_in_ontology()
        self.instances_of_class()
        rospy.spin()

    #############################################
    def handle_insert_object_in_ontology(self, req):
        print 'insert an object into the ontology'
        print req.object.object_id
        print req.object.name
        return InsertObjectResponse(True)
    
    def insert_object_in_ontology(self):
        #rospy.init_node('insert_object_in_ontology_server')
        s = rospy.Service('insert_object_in_ontology', InsertObject, self.handle_insert_object_in_ontology)
        
        print "Ready to insert"

    #############################################
    def handle_instances_of_class(self, class_info):
        print 'list all instances of the same class class_info'
        print class_info.tbox_class.class_id
        print class_info.tbox_class.name
        ret = list()
        ret.append(srs_semantics_db.msg.ABoxObject())

        ### Query... Example here only. need to change
        query = self.prolog.query("owl_subclass_of(A, knowrob:'FoodOrDrink').")
        for sol in query.solutions():
            print 'Found solution. A = %s' %(sol['A'])
        query.finish()

        
        return ListInstancesOfClassResponse(ret)
    
    def instances_of_class(self):
        s = rospy.Service('list_all_instances_of_class', ListInstancesOfClass, self.handle_instances_of_class)
        print 'ready to list instances of one class'
            
    #############################################
"""
    def handle_instance_info(self, object_id):
        print ''
        print class_info.tbox_class.class_id
        print class_info.tbox_class.name
        ret = list()
        ret.append(srs_semantics_db.msg.ABoxObject())
        
        return ListInstancesOfClassResponse(ret)
    
    def instances_of_class(self):
        s = rospy.Service('get_instance_info', ListInstancesOfClass, handle_instance_info)
        print 'ready to list instances of one class'
"""


#############################################

if __name__ == '__main__':
    a = SRSSemanticsDB()
