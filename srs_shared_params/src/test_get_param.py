#!/usr/bin/env python 
import roslib;
roslib.load_manifest('srs_shared_params')
import sys
import rospy

def get_params():
    a = rospy.get_param("/dictionary/a")
    print a
    f = rospy.get_param("/float")
    print f
    d1 = rospy.get_param('/dictionary/a')    
    print d1
    #default_param = rospy.get_param('default_param', 'default_value')

    # fetch a group (dictionary) of parameters
    d2 = rospy.get_param('/dictionary')
    a, c = d2['a'], d2['c']
    print a, c

if __name__ == "__main__":
    get_params()
