#!/usr/bin/env python
import roslib;
roslib.load_manifest('srs_knowledge')
import sys
import rospy

from srs_knowledge.srv import *

def testNextActionService(result):
    print 'Plan next Action service'
    rospy.wait_for_service('plan_next_action')
    try:
        next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)
        
        resp1 = next_action(1, result)
    
        return resp1.nextAction
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def terminateCurrentTask():
    print 'Terminate current task (due to vital problems, e.g. with hardware)'
    rospy.wait_for_service('plan_next_action')
    try:
        next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)
        res = next_action(1, [2, 2, 2])
        print 'here'
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def requestNewTask():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        res = requestNewTask()
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def requestNewTaskMove():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        res = requestNewTask('move', 'kitchen', None, None, None, None)
        print 'send task request'
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def requestNewTaskMove():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        res = requestNewTask('move', '[4 5 6]', None, None, None, None)
        print 'send task request'
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def getObjectsOnMap():
    print 'test get all objects from map'
    try:
        requestNewTask = rospy.ServiceProxy('get_objects_on_map', GetObjectsOnMap)
        res = requestNewTask('ipa-kitchen-map')
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getWorkspaceOnMap():
    print 'test get all workspace (furnitures basically here) from map'
    try:
        requestNewTask = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
        res = requestNewTask('ipa-kitchen-map', True)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":

    # request a new task
    #print requestNewTask()
    print requestNewTaskMove()
    # get next action [0 0 0 ] is not used here as the first step. see explanation in the next call
    print '1-- ', testNextActionService([0,0,0])

    #[0, 0, 0] means: success for move, perception, and grasp actions in the last step

    print '2-- ', testNextActionService([0,0,0])
    #print '3-- ', testNextActionService([0,1,0])
    #print '4-- ', testNextActionService([0,0,0])
    #print '5-- ', testNextActionService([0,0,0])
    #print '6-- ', testNextActionService([0,0,0])
    # to terminate current task, 
    #print terminateCurrentTask()
    print getObjectsOnMap()
    print getWorkspaceOnMap()
