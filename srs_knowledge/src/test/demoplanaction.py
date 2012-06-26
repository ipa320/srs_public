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

def planNextActionService(sessionId, result, feedback):
    print 'Plan next Action service'
    rospy.wait_for_service('plan_next_action')
    try:
        next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)
        
        resp1 = next_action(sessionId, result, feedback)
    
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

def requestNewTaskSearch():
    print 'Request new task - search'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        res = requestNewTask('search', 'Milkbox', 'order')
        print 'send task request'
        return res
    except rospy.ServiceException, e:
        print 'Service call failed: %s'%e

def requestNewTaskJSONMove():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        req = TaskRequestRequest()
        req.task = 'move'
        req.content = ''
        req.json_parameters = '{\"time_schedule\":1263798000000,\"task\":\"move\",\"destination\":{\"predefined_pose\":\"home\"}}'
        req.userPose = ''
        res = requestNewTask(req)
        print 'send task request', req.json_parameters
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def requestNewTaskJSONFetch():
    print 'Request new task'
    rospy.wait_for_service('task_request')
    try:
        requestNewTask = rospy.ServiceProxy('task_request', TaskRequest)
        req = TaskRequestRequest()
        req.task = 'get'
        req.content = 'Milkbox'
        req.json_parameters = '{\"time_schedule\":1263798000000,\"task\":\"get\",\"deliver_destination\":{\"predefined_pose\":\"order\"},\"object\":{\"object_type\":\"Milkbox\"},\"grasping_type\":\"Simple\"}'

        req.userPose = ''
        res = requestNewTask(req)
        print 'send task request', req.json_parameters
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def planNextActionServiceJSON(sessionId, result, jsonFeedback):
    print 'Plan next Action service'
    rospy.wait_for_service('plan_next_action')
    try:
        next_action = rospy.ServiceProxy('plan_next_action', PlanNextAction)
        req = PlanNextActionRequest()
        req.sessionId = sessionId
        req.resultLastAction = result
        req.jsonFeedback = jsonFeedback
        resp1 = next_action(req)
    
        return resp1.nextAction
    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getObjectsOnMap():
    print 'test get all objects from map'
    try:
        requestNewTask = rospy.ServiceProxy('get_objects_on_map', GetObjectsOnMap)
        res = requestNewTask('ipa-kitchen-map', False)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def getWorkspaceOnMap():
    print 'test get all workspace (furnitures basically here) from map'
    try:
        getWorkspace = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
        req = GetWorkspaceOnMapRequest()
        req.map = 'ipa-kitchen-map'
        req.ifGeometryInfo = True
        res = getWorkspace(req)
        #res = getWorkspace('ipa-kitchen-map', True)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def test_fetch_1():
    res = requestNewTaskJSONFetch()
    sessionId = res.sessionId
    acts = list()
    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    jsonFeedback = '{"feedback":{"action":"detect", "object":{"object_type":"Milkbox"}, "pose":{"x":-3.0, "y":-0.2, "z":1.02, "rotx":0, "roty":0, "rotz":0, "rotw":1}}}'
    act = planNextActionServiceJSON(sessionId, 0, jsonFeedback)
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    return acts

def test_move():
    res = requestNewTaskJSONMove()
    sessionId = res.sessionId
    acts = list()
    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    return acts

def test_fetch_2():
    res = requestNewTaskJSONFetch()
    sessionId = res.sessionId
    acts = list()
    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 1, '')
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    jsonFeedback = '{"feedback":{"action":"detect", "object":{"object_type":"Milkbox"}, "pose":{"x":-3.0, "y":-0.2, "z":1.02, "rotx":0, "roty":0, "rotz":0, "rotw":1}}}'
    act = planNextActionServiceJSON(sessionId, 0, jsonFeedback)
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 0, '')
    acts.append(act)

    act = planNextActionServiceJSON(sessionId, 1, '')
    acts.append(act)

    return acts

if __name__ == "__main__":

    # request a new task
    #print requestNewTask()
    #print requestNewTaskSearch()
    # get next action [0 0 0 ] is not used here as the first step. see explanation in the next call
    #print '1-- ', planNextActionService(16, 0, ['s'])

    #[0, 0, 0] means: success for move, perception, and grasp actions in the last step

    #print '2-- ', planNextActionService(16, 1, ['s'])
    #print '3-- ', planNextActionService(16, 1, ['s'])
    #print '4-- ', planNextActionService(16, 1, ['s'])

    #print '4-- ', testNextActionService([0,0,0])
    #print '5-- ', testNextActionService([0,0,0])
    #print '6-- ', testNextActionService([0,0,0])
    # to terminate current task, 
    #print terminateCurrentTask()
    #print getObjectsOnMap()
    #print getWorkspaceOnMap()

    print 'Test FETCH task'
    test_fetch_1()

    print 'Test MOVE task'
    test_move()

    print 'Test FETCH task'
    r = test_fetch_2()
    print r
