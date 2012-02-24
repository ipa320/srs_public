#!/usr/bin/env python
import roslib;
roslib.load_manifest('srs_knowledge')
import sys
import rospy

from srs_knowledge.srv import *
from srs_knowledge.msg import *

def getObjectsOnMap():
    print 'test get all objects from map'
    try:
        getObjects = rospy.ServiceProxy('get_objects_on_map', GetObjectsOnMap)
        res = getObjects('ipa-kitchen-map', True)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getWorkspaceOnMap():
    print 'test get all workspace (furnitures basically here) from map'
    try:
        getWorkspace = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
        res = getWorkspace('ipa-kitchen-map', True)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def updatePosInfo():
    print 'update spatial info'
    try:
        posInfo = SRSSpatialInfo()
        posInfo.pose.position.x = 100
        posInfo.pose.position.y = 1000
        posInfo.pose.position.z = 10000
        posInfo.pose.orientation.x = 0
        posInfo.pose.orientation.y = 0
        posInfo.pose.orientation.z = 0
        posInfo.pose.orientation.w = 1

        posInfo.l = 2
        posInfo.h = 3
        posInfo.w = 4

        updatePos = rospy.ServiceProxy('update_pos_info', UpdatePosInfo)
        res = updatePos('Dishwasher0', 10, posInfo, 'sss')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    # request a new task
    # print requestNewTask()
    # print getObjectsOnMap()
    # print getWorkspaceOnMap()
    print updatePosInfo()
