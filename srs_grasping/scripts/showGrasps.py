#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import sys
import rospy
import openravepy
import grasping_functions

from tf.transformations import *

package_path = roslib.packages.get_pkg_dir('srs_grasping')
##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):
		rospy.init_node("sim_Gzb")
		self.robotName = 'robots/care-o-bot3.zae'


		if (len(sys.argv)<=1):
			self.targetName = 'Milk'
			self.object_path = package_path+"/DB/obj/"+self.targetName+'.xml'

		else:
			self.targetName = sys.argv[1]

			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = package_path+'/DB/obj/'+self.targetName;
			else:
				self.object_path = package_path+'/DB/obj/'+self.targetName+'.xml'

		print "Loaded values: (%s, %s)" %(self.robotName, self.object_path)

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	

		
		env = openravepy.Environment()
 		robot = env.ReadRobotXMLFile(self.robotName)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddRobot(robot)
		env.AddKinBody(target)

	

		file_name = package_path+'/DB/'+self.targetName+"_all_grasps.xml"

		try: 	
			r = raw_input("Do you want to see all the grasps? (y/n): ")
			if r=="y":
				GRASPS = grasping_functions.getGrasps(file_name, all_grasps=True)
			else:
				GRASPS = grasping_functions.getGrasps(file_name)

		except:
			print "There are not generated file."
			sys.exit()


		grasps = GRASPS[0]
		grasping_functions.showOR(env, grasps, gazebo=True, delay=None)

	
		raw_input("Press ENTER to finish.")
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
