#!/usr/bin/env python
import roslib;
roslib.load_manifest('srs_knowledge')
import sys
import rospy

from srs_knowledge.srv import *
from srs_knowledge.msg import *
from srs_msgs.msg import *

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
        posInfo.pose.position.x = 0.65
        posInfo.pose.position.y = 1.21
        posInfo.pose.position.z = 0.1
        posInfo.pose.orientation.x = 0
        posInfo.pose.orientation.y = 0
        posInfo.pose.orientation.z = 0
        posInfo.pose.orientation.w = 1

        posInfo.l = 0.9
        posInfo.h = 0.9
        posInfo.w = 0.74

        updatePos = rospy.ServiceProxy('update_pos_info', UpdatePosInfo)
        res = updatePos('Table0', 7, posInfo, 'not used')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def insertObject():
    print 'update spatial info'
    try:
        insertObj = rospy.ServiceProxy('insert_instance', InsertInstance)
        res = insertObj('Table0', 'Table-PieceOfFurniture', '7', 'not used', 'not used')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def insertMilkbox():
    print 'update spatial info'
    try:
        insertObj = rospy.ServiceProxy('insert_instance', InsertInstance)
        res = insertObj('MilkBox0', 'Milkbox', '9', 'not used', 'not used')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    # print getObjectsOnMap()
    # print getWorkspaceOnMap()
    print insertObject()
    print updatePosInfo()
    print insertMilkbox()
