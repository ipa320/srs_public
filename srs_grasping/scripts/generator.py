#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import sys
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

		self.envName = None

		if (len(sys.argv)<=1):
			self.targetName = 'milk_box'
			self.object_path = package_path+"/DB/obj/"+self.targetName+'.iv'
			print "Loaded default values: (%s, %s)" %(self.robotName, self.object_path)

		else:
			self.targetName = sys.argv[1]

			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = package_path+'/DB/obj/'+self.targetName;
			else:
				self.object_path = package_path+'DB/obj/'+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s, %s)" %(self.robotName, self.object_path)



	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):
	
		env = openravepy.Environment()
    		robot = env.ReadRobotXMLFile(self.robotName)
		env.AddRobot(robot)
		target = env.ReadKinBodyXMLFile(self.object_path)
		env.AddKinBody(target)

		robot.SetActiveManipulator("arm")
		gmodel = openravepy.databases.grasping.GraspingModel(robot=robot,target=target)
		if not gmodel.load():
		    print "GENERATING GRASPS..."
		    gmodel.autogenerate()
		    print "GENERATING GRASPS HAS FINISHED."



		"""
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
		"""
		

		grasping_functions.generateFile(all_grasps=True, targetName=self.targetName, gmodel=gmodel, env=env)


		raw_input("Press ENTER to finish.")
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
