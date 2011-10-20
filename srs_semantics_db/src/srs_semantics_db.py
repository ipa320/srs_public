#!/usr/bin/env python

import roslib
roslib.load_manifest('json_prolog')
import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
    query = prolog.query("member(A, [1,2,3,4]), B = ['x', A]")

    for sol in query.solutions():
        print 'Found solution. A = %s, B = %s' %(sol['A'], sol['B'])

    query = prolog.query("owl_subclass_of(A, knowrob:'FoodOrDrink').")
    for sol in query.solutions():
        print 'Found solution. A = %s' %(sol['A'])
    query.finish()
    
