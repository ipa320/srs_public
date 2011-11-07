#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import actionlib
import sys
import openravepy

from srs_grasping.msg import *
from tf.transformations import *


import grasping_functions

#package_path = rospy.get_param('/repoPath')
package_path = roslib.packages.get_pkg_dir('srs_grasping')+"/DB/"
##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):
		self.robotName = 'robots/care-o-bot3.zae'
		self.client = actionlib.SimpleActionClient('/grasp_server', GraspAction)
		print "Waiting /grasp_server..."
		self.client.wait_for_server()
		print "/grasp_server has been found."

		if (len(sys.argv)<=1):
			self.targetName = 'Milk'
			self.object_path = package_path+"obj/"+self.targetName+'.xml'


		else:
			self.targetName = sys.argv[1]

			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = package_path+'obj/'+self.targetName;
			else:
				self.object_path = package_path+'obj/'+self.targetName+'.xml'


		print "Loaded values: (%s, %s)" %(self.robotName, self.object_path)

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	

		
		env = openravepy.Environment()
		try:
	    		robot = env.ReadRobotXMLFile(self.robotName)
			env.AddRobot(robot)
		except:
			print "The robot file '"+self.robotName+"' does not exists."
			return -1

		try:
			target = env.ReadKinBodyXMLFile(self.object_path)
			env.AddKinBody(target)
		except:
			print "The target file '"+self.targetName+"' does not exists."
			return -1

	
		file_name = package_path+self.targetName+".xml"

		repeat = True
		while repeat:

			print "---- MENU ----"
			print "0 - All the grasps."
			print "1 - An specific grasps for a given axis"
			print "2 - A short number of grasps for a given axis."

			repeat = True
			while repeat:
				r = raw_input(">: ")

				try: 	
					pose_id = ""
					filtered = False

					if r=="0":
						pose_id = ""
						repeat = False

					elif r=="1":
						axis = ""
						while axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
							axis = raw_input("Choose an axis (X, -X, Y, -Y, Z, -Z): ")
							repeat = False
						pose_id = axis

					elif r=="2":
						axis = ""
						while axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
							axis = raw_input("Choose an axis (X, -X, Y, -Y, Z, -Z): ")
							repeat = False
						pose_id = axis

						filtered = True
						repeat = False

					else:
						print "Incorrect option. Try again."


				except:
					print "There are not generated file."
					sys.exit()


				goal = GraspGoal(object_id=0, pose_id=pose_id)
				self.client.send_goal(goal)
				self.client.wait_for_result()
				grasps = (self.client.get_result()).grasp_configuration
				
				if filtered==True:
					grasps = grasping_functions.filterAxis(grasps)


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




		#grasping_functions.showOR(env, grasps, gazebo=gazebo, delay=None)	#GraspConfig
		grasping_functions.showORmsg(env, grasps, gazebo=gazebo, delay=None)	#msg.GraspConfiguration

		return 0
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
    	rospy.init_node('grasp_showGrasps')
	s = SCRIPT()
    	s.run()
