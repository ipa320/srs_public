#!/usr/bin/python

import roslib
roslib.load_manifest('srs_assisted_detection')

from srs_grasping.srv import *
from geometry_msgs.msg import *

def moveBBSrv(req):
    rospy.loginfo("kam was an")
    #save
    rospy.wait_for_service('/object_detection/detect_object')
    try:
        string=String()
        string.data=req.object_name.data
        rospy.loginfo(string)

        srv_get_Objects = rospy.ServiceProxy('/object_detection/detect_object', DetectObjects)
        resp1=srv_get_Objects(req.object_name,roi)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e   
        
    #send action only grasp at the moment   
    rospy.wait_for_service('/get_grasps_from_position')
    try:
        pose=Pose()
        pose.position.x=req.good_object_list.detections[req.id].pose.pose.position.x
        pose.position.y=req.good_object_list.detections[req.id].pose.pose.position.y
        pose.position.z=req.good_object_list.detections[req.id].pose.pose.position.z
        pose.orientation.x=req.good_object_list.detections[req.id].pose.pose.orientation.x
        pose.orientation.y=req.good_object_list.detections[req.id].pose.pose.orientation.y
        pose.orientation.z=req.good_object_list.detections[req.id].pose.pose.orientation.z
        pose.orientation.w=req.good_object_list.detections[req.id].pose.pose.orientation.w


        srv_get_Objects = rospy.ServiceProxy('/get_grasps_from_position', GetGraspsFromPosition)
        resp1=srv_get_Objects(0,pose)
        #rospy.loginfo(resp1.object_list.detections[0])    
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e   
        
def asisted_BBmove_server():
    rospy.init_node('asisted_BBmove_server')
    s = rospy.Service('asisted_BBmove', BBMove, moveBBSrv)


    rospy.loginfo("Assisted BBmove ready.")
    rospy.spin()
        

if __name__ == "__main__":
    asisted_BBmove_server()