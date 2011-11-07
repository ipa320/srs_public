#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import sys
import openravepy
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


		robot.SetActiveManipulator("arm")		#care-o-bot3.zae
		gmodel = openravepy.databases.grasping.GraspingModel(robot=robot,target=target)
		if not gmodel.load():
		    print "GENERATING GRASPS..."
		    gmodel.autogenerate()
		    print "GENERATING GRASPS HAS FINISHED."



		grasping_functions.generateFile(targetName=self.targetName, gmodel=gmodel, env=env)
		return 0
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	rospy.init_node('grasp_generator')
	s = SCRIPT()
    	s.run()
