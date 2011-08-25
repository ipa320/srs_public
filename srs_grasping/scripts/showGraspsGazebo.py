#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import roslib.packages
import rospy
import sys
from trajectory_msgs.msg import *
import grasping_functions


package_path = roslib.packages.get_pkg_dir('srs_grasping')


class SCRIPT():

	def __init__(self):

		rospy.init_node("simGaz_script")

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





	def run(self):

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

	
		repeat = True;
		while repeat:
			#conf = int(raw_input("There are "+str(len(GRASPS))+" configurations. What do you want to see?: "));
			conf=0
			if (conf>=len(GRASPS)):
				continue;
			g = GRASPS[conf]
			print str(len(g))+" grasps will be showed.";

			for i in range (0,len(g)):
				print 'grasp %d/%d'%(i,len(g))
				grasping_functions.Grasp(g[i].joint_values)
				raw_input("...")

			r = raw_input("Do you want to see another configuration? (y/n): ");
			if r=="y":
				repeat = True;
			else:
				repeat = False;





if __name__ == "__main__":
	s = SCRIPT()
	s.run()
