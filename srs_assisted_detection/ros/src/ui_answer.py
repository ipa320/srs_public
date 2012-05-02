#!/usr/bin/python

import roslib
roslib.load_manifest('srs_assisted_detection')

#from srs_grasping.srv import *
from geometry_msgs.msg import *

from sensor_msgs.msg import *
from srs_assisted_detection.srv import *
from geometry_msgs.msg import *


import rospy
detect=UiDetectorResponse()
func=0
pose=Pose()
action=''
def answerObjectSrv(req):    

    rospy.loginfo("Get Object information")

    global func
    func=1
        

    #save
    
        
    #send action only grasp at the moment   
  
    global pose
    pose.position.x=detect.object_list.detections[req.id].pose.pose.position.x
    pose.position.y=detect.object_list.detections[req.id].pose.pose.position.y
    pose.position.z=detect.object_list.detections[req.id].pose.pose.position.z
    pose.orientation.x=detect.object_list.detections[req.id].pose.pose.orientation.x
    pose.orientation.y=detect.object_list.detections[req.id].pose.pose.orientation.y
    pose.orientation.z=detect.object_list.detections[req.id].pose.pose.orientation.z
    pose.orientation.w=detect.object_list.detections[req.id].pose.pose.orientation.w
    global action
    
    #default for user
    action='grasp'

    s.shutdown()
    s2.shutdown()
    moveBB=UiAnswerResponse()
    moveBB.message.data='get data waiting for action'
    return moveBB

        
        
        
def moveBBSrv(req):
    rospy.loginfo("Get BB information")

    #BBmove service base and then movement
    moveBB=BBMoveResponse()
    moveBB.message.data='moving to better position'
    global func
    func=2
    
    global pose
    pose.position.x=1
    pose.position.y=2
    pose.position.z=3
    
    s.shutdown()
    s2.shutdown()
    return moveBB
    
    
    
    
    
def assisted_answer_server(detections):
    #rospy.init_node('asisted_answer_server')
    global detect
    detect=detections
    global s
    global s2
    s = rospy.Service('assisted_answer', UiAnswer, answerObjectSrv)
    s2 = rospy.Service('assisted_BBmove', BBMove, moveBBSrv)

    rospy.loginfo("Assisted answer ready.")
    s.spin()
    s2.spin()
    return [func,pose,action]
if __name__ == "__main__":
    assisted_answer_server('test')