#!/usr/bin/env python
import roslib;
#roslib.load_manifest('beginner_tutorials')
roslib.load_manifest('srs_knowledge')
roslib.load_manifest('geometry_msgs')
import sys
import rospy
from geometry_msgs.msg import *

from srs_knowledge.srv import *

def querySparQL():
    rospy.wait_for_service('query_sparql')
    try:
        print '\n---- Try SparQL to query all instances of type Table ----'
        spql = rospy.ServiceProxy('query_sparql', QuerySparQL)
        queryString = "PREFIX house: <http://www.semanticweb.org/ontologies/house.owl#> PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> \n SELECT ?table \n WHERE { ?table rdf:type house:Table . }"
        print queryString
	print '----\n'
	resp1 = spql(queryString)
        return resp1.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

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
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def requestNewTask():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
	p = Pose2D()
	p.x = 1
	p.y = 1
	p.theta = 0
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        res = requestNewTask('get', 'MilkBox', None)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print querySparQL()
    #print requestNewTask()
    #print testNextActionService([0,0,0])
    #print testNextActionService([0,0,0])
    #print testNextActionService([0,1,0])
    #print testNextActionService([0,0,0])
    #print testNextActionService([0,0,0])
