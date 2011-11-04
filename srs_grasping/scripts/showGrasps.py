#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')

import sys
import openravepy

from tf.transformations import *


import grasping_functions
package_path = roslib.packages.get_pkg_dir('srs_grasping')

##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):
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

		repeat = True
		while repeat:

			print "---- MENU ----"
			print "0 - All the grasps."
			print "1 - Specific grasps."
			print "2 - A short number of grasps for each axis."

			repeat = True
			while repeat:
				r = raw_input(">: ")
				try: 	
					if r=="0":
						grasps = grasping_functions.getGrasps(file_name, all_grasps=True)
						repeat = False
					elif r=="1":
						GRASPS = grasping_functions.getGrasps(file_name, all_grasps=True)
						axis = ""
						while axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
							axis = raw_input("Choose an axis (X, -X, Y, -Y, Z, -Z): ")
							grasps = grasping_functions.getGraspsByAxis(GRASPS, axis)
							repeat = False
					elif r=="2":
						grasps = grasping_functions.getGrasps(file_name)
						repeat = False
					else:
						print "Incorrect option. Try again."

				except:
					print "There are not generated file."
					sys.exit()
				
			rep = True
			while rep:
				gaz = raw_input("Do you want to see the grasps in gazeboo? (y/n): ")
				if gaz=="y":
					gazebo = True
					rep = False
				elif gaz=="n":
					gazebo = False
					rep = False
				else:
					print "Incorrect option. Try again."






		grasping_functions.showOR(env, grasps, gazebo=gazebo, delay=None)
		raw_input("Press ENTER to finish.")
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
