#!/usr/bin/env python

import generator, showGrasps, grasp_machine

##################################################################################	
class SCRIPT():###################################################################
##################################################################################
	
	def __init__(self):
		print "Running the assistent."
	
	def run(self):	
			print "<-------------- MENU -------------->"
			print "1 - Generator."
			print "2 - Show grasps."
			print "3 - Grasp machine."
			print "4 - Help."

			repeat = True;
			while repeat:
				try:
					option = int(raw_input("?: "));
					if (option>=1 and option<=4):
						repeat = False;
					else:
						print "Wrong value."
				except:
					print "Wrong value."

	
			if option == 1:
				s = generator.SCRIPT()
				s.run()

			elif option == 2:
				s = showGraspsOR.SCRIPT()
				s.run()
			elif option == 3:
				grasp_machine.main()
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
