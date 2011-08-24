#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy

import grasping_functions
from srs_grasping.msg import *
from srs_grasping.srv import *



def GetGraspsFunction(req):

	package_path = roslib.packages.get_pkg_dir('srs_grasping')
	file_name = package_path+'/DB/'+req.ObjectID+"_all_grasps.xml"
	res = GetGraspsResponse()
	
	try:
		GRASPS = grasping_functions.getGrasps(file_name, pose=req.poseID, msg=True)
		rospy.loginfo(str(len( GRASPS[0]))+" grasping configuration for this object.")			

		g = GRASPS[0]

		if len(g)==0:

			print "----------------------------"
			print "Wrong poseID value. Options:"
			print "		TOP"
			print "		DOWN"
			print "		SIDEX"
			print "		SIDEmX"
			print "		SIDEY"
			print "		SIDEmY"
			print "----------------------------"

			res.grasps = []
			res.response = False;	
			return res

		res.grasps = g
		res.response = True;

	except:
		rospy.logerr("No grasping configurations for "+req.ObjectID+" object.")	
		res.grasps = []
		res.response = False;		

	return res
    

def GetGraspsService():

	rospy.init_node("GetGraspsService")
	s = rospy.Service('/grasp_service/GetGrasps', GetGrasps, GetGraspsFunction)
	print "/grasp_service/GetGrasps service is running."
	rospy.spin()



if __name__ == "__main__":
	GetGraspsService()
