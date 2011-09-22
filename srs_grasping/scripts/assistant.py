#!/usr/bin/env python

import generator, depurator, showGraspsOR, showGrasps

##################################################################################	
class SCRIPT():###################################################################
##################################################################################
	
	def __init__(self):
		print "Running the assistent."
	
	def run(self):	
			print "<-------------- MENU -------------->"
			print "0 - Generator"
			print "1 - Depurator <obsolet>"
			print "2 - Show grasps in OpenRAVE"
			print "3 - Show grasps in OpenRAVE and Gazebo"
			print "4 - Help"

			repeat = True;
			while repeat:
				try:
					option = int(raw_input("?: "));
					if (option>=0 and option<=4):
						repeat = False;
					else:
						print "Wrong value."
				except:
					print "Wrong value."

	
			if option == 0:
				s = generator.SCRIPT()
				s.run()

			elif option == 1:
				res = raw_input("The script is obsolet. Do you want to continue? (y/n): ")
				if res=="y":
					s = depurator.SCRIPT()
					s.run()

			elif option == 2:
				s = showGraspsOR.SCRIPT()
				s.run()

			elif option == 3:
				s = showGrasps.SCRIPT()
				s.run()

			else:
				print "USE: "
				print "     rosrun srs_grasping assistant.py (Load Milk.xml value)"
				print "     rosrun srs_grasping assistant.py object (Load object.xml value)"
				print "     rosrun srs_grasping assistant.py object.iv (Load object.iv value)"



##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
