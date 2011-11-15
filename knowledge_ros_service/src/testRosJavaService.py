#!/usr/bin/env python
import roslib;
#roslib.load_manifest('beginner_tutorials')
roslib.load_manifest('knowledge_ros_service')
import sys
import rospy

from knowledge_ros_service.srv import *

def add_two_ints_client():
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AskForActionSequence)
        
        resp1 = add_two_ints('get', 'milk0', None, None)
        #resp1 = add_two_ints(x, y)
        print 'Service call ...'
        return resp1.actionSequence
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def querySparQL():
    rospy.wait_for_service('query_sparql')
    try:
        print 'ssss'
        spql = rospy.ServiceProxy('query_sparql', QuerySparQL)
        queryString = "PREFIX house: <http://www.semanticweb.org/ontologies/house.owl#> PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n SELECT ?room \n WHERE { ?room rdf:type house:Table . }"
        resp1 = spql(queryString)
        #resp1 = add_two_ints(x, y)
        print 'Service call ...'
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def testNextActionService(result):
    print 'Plan next Action service'
    rospy.wait_for_service('plan_next_action')
    try:
        next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)
        print 'here'
        
        resp1 = next_action(1, result)
        print 'here'
    
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


def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    #print len(add_two_ints_client())
    #print querySparQL()
    print requestNewTask()
    print testNextActionService([0,0,0])
    print testNextActionService([0,0,0])
    print testNextActionService([0,1,0])
    print testNextActionService([0,0,0])
    print testNextActionService([0,0,0])
