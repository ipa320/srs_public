#!/usr/bin/python

import roslib
roslib.load_manifest('srs_assisted_detection')

#from srs_grasping.srv import *
from geometry_msgs.msg import *
from srs_assisted_detection.srv import *

from srs_symbolic_grounding.srv import *
from srs_symbolic_grounding.msg import *
from srs_msgs.msg import * # this is for SRSSpatialInfo()

import rospy

def moveBBSrv(req):
    
        try:
            rospy.wait_for_service('scan_base_pose',10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            outcome_detectObjectSrv = 'failed'
        
        try:
            base_pose_service = rospy.ServiceProxy('scan_base_pose', ScanBasePose)
            req_scan = ScanBasePoseRequest()
            srs_info=SRSSpatialInfo()
            srs_info.l=req.l
            srs_info.w=req.w
            srs_info.h=req.h
            srs_info.pose=req.pose 
            req_scan.parent_obj_geometry=srs_info
            res = base_pose_service(req_scan)
            print res.scan_base_pose_list[0]
            moveBB=BBMoveResponse()
            moveBB.message.data='moving to better position'
            return moveBB
            outcome_user_intervention = 'succeeded'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
def asisted_BBmove_server():
    rospy.init_node('asisted_BBmove_server')
    s = rospy.Service('assisted_BBmove', BBMove, moveBBSrv)


    rospy.loginfo("Assisted BBmove ready.")
    rospy.spin()
        

if __name__ == "__main__":
    asisted_BBmove_server()