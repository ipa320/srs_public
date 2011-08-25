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
		print "-----------------_"
		rospy.loginfo(str(len( GRASPS[0]))+" grasping configuration for this object.")			


		g = GRASPS[0]
		if len(g)==0:

			print "----------------------------"
			print "Wrong poseID value. Options:"
			print "		 Z"
			print "		_Z"
			print "		 X"
			print "		_X"
			print "		 Y"
			print "		_Y"
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

	rospy.init_node("get_grasp_service_server")
	s = rospy.Service('/grasp_service/get_grasps', GetGrasps, GetGraspsFunction)
	print "/grasp_service/get_grasps service is running."
	rospy.spin()



if __name__ == "__main__":
	GetGraspsService()
