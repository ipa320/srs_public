#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import sys
from tf.transformations import *
import openravepy
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
			self.targetName = 'milk_box'
			self.object_path = package_path+"/DB/obj/"+self.targetName+'.iv'
			print "Loaded default values: (%s, %s)" %(self.robotName, self.object_path)
		else:
			self.targetName = sys.argv[1]
			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = package_path+'/DB/obj/'+self.targetName;
			else:
				self.object_path = package_path+'/DB/obj/'+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s, %s)" %(self.robotName, self.object_path)

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	

		
		env = openravepy.Environment()
 		robot = env.ReadRobotXMLFile(self.robotName)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddRobot(robot)
		env.AddKinBody(target)

	
		#r2 = raw_input("Do you want to see the depurated file? (y/n): ");
		r2="n"
		if r2=="y":
			file_name = package_path+'/DB/'+self.targetName+"_D.xml"
		else:
			file_name = package_path+'/DB/'+self.targetName+"_all_grasps.xml"



		try: 
			REP = True
			while REP:
				print "What configuration do you prefer?"
				print " X  Y  Z (positive axis)"
				print "_X _Y _Z (negative axis)"
				res = raw_input("?: ")
				if res!="X" and res!="_X" and res!="Y" and res!="_Y" and res!="Z" and res!="_Z":
					print "Wrong value."
				else: 
					GRASPS = grasping_functions.getGrasps(file_name, res)
					REP = False;

		except:
			print "There are not generated file."
			sys.exit()

		repeat=True;
		while repeat==True:

			rep = True;
			conf = 0;
			while rep:
				try:
					#conf = int(raw_input("There are "+str(len(GRASPS))+" configurations. What do you want to see?: "))
					conf = 0

					if (conf<0 or conf>=len(GRASPS)):
						print "Wrong configuration."
						rep = True;
					else:
						rep = False;
				except:
					print "Wrong configuration."
					rep = True;			




			grasps = GRASPS[conf]
			g = grasping_functions.showOR(env, grasps, delay=None)

			res = raw_input("Do you want to see another configuration? (y/n): ")
			if res=="n":
				repeat = False
	
						
		raw_input("Press ENTER to finish.")
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
