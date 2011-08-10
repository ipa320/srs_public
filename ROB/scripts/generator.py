#!/usr/bin/env python

import roslib; 
roslib.load_manifest('ROB')
import rospy
import os,sys,itertools,traceback,time, random
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

		self.collisionCheckerName =  "bullet"
		self.robotName = 'robots/schunk-sdh.zae'
		self.envName = None
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
		cc = RaveCreateCollisionChecker(env, self.collisionCheckerName)
		env.SetCollisionChecker(cc)



    		robot = env.ReadRobotXMLFile(self.robotName)
		env.AddRobot(robot)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddKinBody(target)


		gmodel = databases.grasping.GraspingModel(robot=robot,target=target)
		if not gmodel.load():
		    print "GENERATING GRASPS..."
		    gmodel.autogenerate()
		    print "GENERATING GRASPS HAS FINISHED."



		res = raw_input("Do you want to load a collision environment? (y/n): ")
		if res=="y":
			env.Remove(target)
			self.envName = raw_input("Write the environment file absolute path: ")

			while(not env.Load(self.envName)):
				print "File not found."
				self.envName = raw_input("Write the environment file absolute path: ")

			all_grasps = False
		else:
			print "A free collision environment will be generated."
			all_grasps = True

		

		grasp_functions.generaFicheroXML(all_grasps=all_grasps, targetName=target.GetName(), realTargetName=self.targetName, gmodel=gmodel, envName = self.envName)

		raw_input("Press ENTER to finish.")



##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
