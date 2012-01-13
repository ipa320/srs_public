#!/usr/bin/env python

import roslib; 
roslib.load_manifest('srs_grasping')
import rospy
import actionlib
import sys, os
import openravepy

from srs_grasping.msg import *
from tf.transformations import *


import grasping_functions
from srs_object_database.msg import *
from srs_object_database.srv import *


##################################################################################	
class SCRIPT():###################################################################
##################################################################################	

	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def __init__(self):

		self.robotName = 'robots/care-o-bot3.zae'


	# ------------------------------------------------------------------------------------
	# ------------------------------------------------------------------------------------
	def run(self):	

		env = openravepy.Environment()

		try:
	    		robot = env.ReadRobotXMLFile(self.robotName)
			env.AddRobot(robot)
		except:
			rospy.logerr("The robot file %s does not exists.", self.robotName)
			return -1


		object_id = (sys.argv[len(sys.argv)-1], 1)[len(sys.argv)==1]	#Operador ternario

		rospy.loginfo("Waiting /get_model_mesh service...")
		rospy.wait_for_service('/get_model_mesh')
		rospy.loginfo("/get_model_mesh has been found!")

		get_mesh = rospy.ServiceProxy('/get_model_mesh', GetMesh)
		try:
			resp = get_mesh(model_ids=[object_id])
		except rospy.ServiceException, e:
			rospy.logerr("Service did not process request: %s", str(e))
			return -1

		try:
			mesh_file = "/tmp/mesh.iv"
			f = open(mesh_file, 'w')
			f.write(resp.msg[0].data)
			f.close()
			target = env.ReadKinBodyXMLFile(mesh_file)
			env.AddKinBody(target)
			os.remove(mesh_file)
		except:
			rospy.logerr("The mesh data does not exist or does not work correctly.")
			os.remove(mesh_file)
			return -1



		

		repeat = True
		while repeat:

			print "---- MENU ----"
			print "0 - All the grasps."
			print "1 - An specific grasps for a given axis"
			print "2 - A short number of grasps for a given axis."

			repeat = True
			while repeat:
				r = raw_input(">: ")

				try: 	
					pose_id = ""
					filtered = False

					if r=="0":
						pose_id = ""
						repeat = False

					elif r=="1":
						axis = ""
						while axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
							axis = raw_input("Choose an axis (X, -X, Y, -Y, Z, -Z): ")
							repeat = False
						pose_id = axis

					elif r=="2":
						axis = ""
						while axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
							axis = raw_input("Choose an axis (X, -X, Y, -Y, Z, -Z): ")
							repeat = False
						pose_id = axis

						filtered = True
						repeat = False

					else:
						print "Incorrect option. Try again."


				except:
					print "There are not generated file."
					sys.exit()



				client = actionlib.SimpleActionClient('/grasp_server', GraspAction)
				rospy.loginfo("Waiting /grasp_server...")
				client.wait_for_server()
				rospy.loginfo("/grasp_server has been found!")

				goal = GraspGoal(object_id=object_id, pose_id=pose_id)
				client.send_goal(goal)
				client.wait_for_result()
				grasps = (client.get_result()).grasp_configuration
				
				if filtered==True:
					grasps = grasping_functions.filterAxis(grasps)


			rep = True
			while rep:
				gaz = raw_input("Do you want to see the grasps in gazeboo? (y/n): ")

				if gaz=="y":
					gazebo = True
					rep = False
				elif gaz=="n":
					gazebo = False
					rep = False
				else:
					print "Incorrect option. Try again."



		grasping_functions.showORmsg(env, grasps, gazebo=gazebo, delay=None)

		return 0
		


##########################################################################
if __name__ == "__main__":################################################
##########################################################################
    	rospy.init_node('grasp_showGrasps')
	s = SCRIPT()
    	s.run()
