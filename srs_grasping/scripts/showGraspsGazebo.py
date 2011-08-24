#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import roslib.packages
import rospy
import sys
import simple_script_server
from trajectory_msgs.msg import *
import grasping_functions


package_path = roslib.packages.get_pkg_dir('srs_grasping')


class SCRIPT():

	def __init__(self):


		if (len(sys.argv)<=1):
			self.targetName = 'milk_box'
			self.object_path = package_path+"/DB/obj/"+self.targetName+'.iv'
			print "Loaded default values: (%s)" %(self.object_path)
		else:
			self.targetName = sys.argv[1]
			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = package_path+'/DB/obj/'+self.targetName;
			else:
				self.object_path = package_path+'/DB/obj/'+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s)" %(self.object_path)



		rospy.init_node("simGaz_script")
		print "Loading script server..."
		self.sss = simple_script_server.simple_script_server()
		print "Script server loaded."

	def run(self):

		#r2 = raw_input("Do you want to see the depurated file? (y/n): ");
		r2="n"
		if r2=="y":
			file_name = package_path+'/DB/'+self.targetName+"_D.xml"
		else:
			file_name = package_path+'/DB/'+self.targetName+"_all_grasps.xml"


		try:
			grasps = grasping_functions.getGrasps(file_name, "Z");
		except:
			print "There are not generated file."
			sys.exit()	

	
		repeat = True;
		while repeat:
			#conf = int(raw_input("There are "+str(len(grasps))+" configurations. What do you want to see?: "));
			conf=0
			if (conf>=len(grasps)):
				continue;
			g = grasps[conf]
			print str(len(g))+" grasps will be showed.";

			for i in range (0,len(g)):
				self.sss.move("sdh",[eval(g[i].joint_values)])
				self.sss.wait_for_input()


			r = raw_input("Do you want to see another configuration? (y/n): ");
			if r=="y":
				repeat = True;
			else:
				repeat = False;





if __name__ == "__main__":
	s = SCRIPT()
	s.run()
