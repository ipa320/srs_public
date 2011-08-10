#!/usr/bin/env python

import roslib; 
roslib.load_manifest('ROB')
import rospy
import os,sys,itertools,traceback,time
import grasp_functions
from numpy import *
from openravepy import *



##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):
		print "---------------------------------------------"
		print "To run this script you must be in his folder." 
		print "---------------------------------------------"

		self.robotName = 'robots/schunk-sdh.zae'
		if (len(sys.argv)<=1):
			self.targetName = 'mug'
			self.object_path = '../DB/obj/'+self.targetName+'.kinbody.xml'
			print "Loaded default values: (%s, %s)" %(self.robotName, self.object_path)
		else:
			self.targetName = sys.argv[1]

			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = '../DB/obj/'+self.targetName;
			else:
				self.object_path = '../DB/obj/'+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s, %s)" %(self.robotName, self.targetName)

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	
		

		env = Environment()
 		robot = env.ReadRobotXMLFile(self.robotName)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddRobot(robot)
		env.AddKinBody(target)


		env.SetViewer('qtcoin')
		time.sleep(1.0)


	
		r2 = raw_input("Do you want to see the depurated file? (y/n): ");
		if r2=="y":
			file_name = self.targetName+"_D.xml"
		else:
			file_name = self.targetName+"_all_grasps.xml"



		repeat=True;
		conf = 0;
		while repeat==True:
			try: 
				GRASPS = grasp_functions.getGrasps(file_name)
			except:
				print "There are not generated file."
				sys.exit()
		
			rep = True;
			while rep:
				try:
					conf = int(raw_input("There are "+str(len(GRASPS))+" configurations. What do you want to see?: "))
					if (conf<0 or conf>=len(GRASPS)):
						print "Wrong configuration."
						rep = True;
					else:
						rep = False;
				except:
					print "Wrong configuration."
					rep = True;			




			grasps = GRASPS[conf]
			g = grasp_functions.showOR(env=env, grasps=grasps, delay=None)


			res = raw_input("Do you want to see another configuration? (y/n): ")
			if res=="n":
				repeat = False
	
						
		raw_input("Press ENTER to finish.")



##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
