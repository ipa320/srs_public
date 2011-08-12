#!/usr/bin/env python

import roslib; 
roslib.load_manifest('ROB')
import rospy
import sys
import grasp_functions
import simple_script_server
from trajectory_msgs.msg import *

class SCRIPT():

	def __init__(self):
		print "---------------------------------------------"
		print "To run this script you must be in his folder." 
		print "---------------------------------------------"

		if (len(sys.argv)<=1):
			self.targetName = 'mug'
			self.object_path = '../DB/obj/'+self.targetName+'.kinbody.xml'
			print "Loaded default values: (%s)" %(self.object_path)
		else:
			self.targetName = sys.argv[1]

			if self.targetName[len(self.targetName)-3:len(self.targetName)] == ".iv":
				self.object_path = '../DB/obj/'+self.targetName;
			else:
				self.object_path = '../DB/obj/'+self.targetName+'.kinbody.xml'

			print "Loaded values: (%s, %s)" %(self.robotName, self.targetName)



		rospy.init_node("simGaz_script")
		self.sss = simple_script_server.simple_script_server()


	def run(self):

		r2 = raw_input("Do you want to see the depurated file? (y/n): ");
		if r2=="y":
			file_name = self.targetName+"_D.xml"
		else:
			file_name = self.targetName+"_all_grasps.xml"


		try:
			grasps = grasp_functions.getGrasps(file_name);
		except:
			print "There are not generated file."
			sys.exit()	

	
		repeat = True;
		while repeat:
			conf = int(raw_input("There are "+str(len(grasps))+" configurations. What do you want to see?: "));
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
