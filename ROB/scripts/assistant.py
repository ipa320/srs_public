#!/usr/bin/env python

import generator, depurator, showGraspsOR, showGraspsGazebo

##################################################################################	
class SCRIPT():###################################################################
##################################################################################
	
	def __init__(self):
		print "Running assistant."
	
	def run(self):	
		#finish = False;
		#while not finish:

			print "<-------------- MENU -------------->"
			print "0 - Generate grasp configurations."
			print "1 - Depurate manually the XML file."
			print "2 - Show grasps in OpenRAVE"
			print "3 - Show grasps in Gazebo"
			print "4 - Help"
			print "<---------------------------------->"

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
				s = depurator.SCRIPT()
				s.run()

			elif option == 2:
				s = showGraspsOR.SCRIPT()
				s.run()

			elif option == 3:
				s = showGraspsGazebo.SCRIPT()
				s.run()

			else:
				print "USE: "
				print "     ./assistant.py (load default values)"
				print "     ./assistant.py object (load ../DB/obj/object.kinbody.xml value)"
				print "     ./assistant.py object.iv (load ../DB/obj/object.iv value)"
				print "     ------------------------------------------------------\n"
				
				print "     ROB/DB folder:         Grasping configuration files."
				print "     ROB/scripts folder:    Scripts."
				print "     ROB/DB/obj folder:     OpenRAVE object files."
				print "     ------------------------------------------------------\n"
	

			#res = raw_input("Finish? (y/n): ");
			#if res=="y":
			#	finish = True;




##########################################################################
if __name__ == "__main__":################################################
##########################################################################
	s = SCRIPT()
    	s.run()
