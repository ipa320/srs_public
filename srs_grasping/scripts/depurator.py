#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import sys
import openravepy
import grasping_functions

package_path = roslib.packages.get_pkg_dir('srs_grasping');

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
				self.object_path = package_path+"/DB/obj/"+self.targetName;
			else:
				self.object_path =package_path+"/DB/obj/"+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s, %s)" %(self.robotName, self.object_path)

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	
		

		env = openravepy.Environment()
 		robot = env.ReadRobotXMLFile(self.robotName)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddRobot(robot)
		env.AddKinBody(target)



		repetir=True
		while repetir==True:
			GRASPS_D = []
			try:
				GRASPS = grasping_functions.getGrasps(package_path+"/DB/"+self.targetName+"_all_grasps.xml", "Z")
			except:
				print "There are not generated file."
				sys.exit()
	
			print "There are "+str(len(GRASPS))+" configurations for the object "+self.targetName

			try:
				GRASPS_D = grasping_functions.getGrasps(package_path+"/DB/"+self.targetName+"_D.xml", "Z")
			except:
				print "Any configurations has been depurated."



			if len(GRASPS_D) < len(GRASPS):
				print "The configuration "+str(len(GRASPS_D))+" will be depurated.";
				g = grasping_functions.showOR(env=env, grasps=GRASPS[len(GRASPS_D)], depurador=True)
				if len(g)>0:
					grasping_functions.generaFicheroDepurado(targetName=self.targetName, grasps=g)
					res = raw_input("Do you want to show another configuration? (y/n): ")
					if res=="n":
						repetir = False
			else:
				print "All the configurations has been depurated."
				repetir = False;

	
						
		
		raw_input("Press ENTER to finish.")



##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
