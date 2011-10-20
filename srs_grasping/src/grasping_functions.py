#!/usr/bin/env python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import sys, time
import openravepy

from xml.dom import minidom
from numpy import *

from tf.transformations import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from srs_grasping.msg import *
from srs_msgs.msg import *

from simple_script_server import *
sss = simple_script_server()

pi = math.pi
package_path = roslib.packages.get_pkg_dir('srs_grasping')


#################################################################################################
class GraspConfig(): ############################################################################
#################################################################################################

	def __init__(self, object_id, score, category, sconfiguration, palm_pose, pre_grasp):
		self.object_id = object_id
		self.hand_type = "sdh"
		self.sconfiguration = sconfiguration
		self.palm_pose = palm_pose
		self.pre_grasp = pre_grasp
		self.category = category
		self.score = score


	
	def __cmp__(self,other):
		
		if self.score > other.score:
			return 1
		elif self.score < other.score:
			return -1
		else:
			return 0
		

	###### [ GRASP CMP FILTERS ] ##################################################
	def Z(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[2][2]
		o = mo[2][2]

		if s > o:
			return -1
		elif  s < o:
			return 1
		else:
			return 0


	def _Z(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[2][2]
		o = mo[2][2]

		if s > o:
			return 1
		elif  s < o:
			return -1
		else:
			return 0


	def X(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[0][2]
		o = mo[0][2]

		if s > o:
			return -1
		elif  s < o:
			return 1
		else:
			return 0


	def _X(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[0][2]
		o = mo[0][2]

		if s > o:
			return 1
		elif  s < o:
			return -1
		else:
			return 0


	def Y(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[1][2]
		o = mo[1][2]

		if s > o:
			return -1
		elif  s < o:
			return 1
		else:
			return 0


	def _Y(self,other):
		ms = matrix_from_graspPose(self.palm_pose)
		mo = matrix_from_graspPose(other.palm_pose)
		s = ms[1][2]
		o = mo[1][2]

		if s > o:
			return 1
		elif  s < o:
			return -1
		else:
			return 0


	def mZ(self,other):
		s = self.palm_pose.pose.position.z
		o = other.palm_pose.pose.position.z

		if s > o:
			return -1
		elif  s < o:
			return 1
		else:
			return 0

	###### [/GRASP FILTERS ] ##################################################


#################################################################################################
###################################### CONVERTERS ###############################################
#################################################################################################
# Convert OpenRAVE to Gazebo format.
def OR_to_ROS(OR):
	return [OR[2], OR[3], OR[4], OR[0], OR[1], OR[5], OR[6]]


# Convert Gazebo to OpenRAVE format.
def ROS_to_OR(ROS):
	return [ROS[3], ROS[4], ROS[0], ROS[1], ROS[2], ROS[5], ROS[6]]


# Convert /sdh_controller/command to script_server/sdh/joint_names
def ROS_to_script_server(values):
	return [values[0], values[5], values[6], values[1], values[2], values[3], values[4]]
	
# Convert /script_server/sdh/ to OpenRAVE
def COB_to_OR(values):

	values = [values[0], values[3], values[4], values[5], values[6], values[1], values[2]]
	values = ROS_to_OR(values)
	return values
	

# Convert GraspConfig to msg format.
def graspConfig_to_MSG(res):
	aux = []

	for i in range(0,len(res)):

		jt = JointTrajectory()
		jt.joint_names = ["sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint", "sdh_finger_23_joint"]
		jt.points = []
		p = JointTrajectoryPoint()
		p.positions = eval(res[i].sconfiguration)
		jt.points.append(p)
	
		GC = GraspConfiguration()
		GC.object_id = hash(res[i].object_id)
		GC.hand_type = str(res[i].hand_type)
		GC.palm_pose = res[i].palm_pose
		GC.pre_grasp = res[i].pre_grasp
		GC.sconfiguration = jt
		GC.category= str(res[i].category)
		GC.score =  float(res[i].score)

		aux.append(GC)

	return aux


# Convert GraspConfig to matrix.
def matrix_from_graspPose(gp):

	q = []
	q.append(gp.pose.orientation.x)
	q.append(gp.pose.orientation.y)
	q.append(gp.pose.orientation.z)
	q.append(gp.pose.orientation.w)
	e = euler_from_quaternion(q, axes='sxyz')

	matrix = euler_matrix(e[0],e[1],e[2] ,axes='sxyz')
	matrix[0][3] = gp.pose.position.x
	matrix[1][3] = gp.pose.position.y
	matrix[2][3] = gp.pose.position.z

	return matrix


# Convert string list to float list (needed for OR).
def stringList_to_ORformat(OR):
	res = [0 for i in range(28)]	

	#OR = ROS_to_OR(eval(OR))
	OR = COB_to_OR(eval(OR))

	for i in range (0,len(OR)):
		res[i+7] = float(OR[i])

	return res


#################################################################################################
############################## GRASPING FUNCTIONS ###############################################
#################################################################################################
# -----------------------------------------------------------------------------------------------
# XML file generator.
# -----------------------------------------------------------------------------------------------
def generateFile(targetName, gmodel, env):

	# ------------------------ 
	# <targetName_all_grasps>.xml
	# ------------------------ 

	f_name = targetName + "_all_grasps.xml"

	try:
		f = open(package_path+'/DB/'+f_name,'r')
		print "There are a file with the same name. It will be rewritted."
		raw_input("Continue...")
		sys.exit()
	except:
		f = open(package_path+'/DB/'+f_name,'w')
		f.write("<?xml version=\"1.0\" ?>\n")
		f.write("<GraspList>\n")
		f.write("<object_id>"+targetName+"</object_id>\n")
		f.write("<hand_type>SDH</hand_type>\n")
		f.write("<joint_names>[sdh_knuckle_joint, sdh_thumb_2_joint, sdh_thumb_3_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint]</joint_names>\n")
		f.write("<grasp_reference_link>sdh_palm_link</grasp_reference_link>\n")
		#f.write("<pose_reference_link>base_link</pose_reference_link>\n")
		f.write("<configuration id=\"0\">\n")


		cont = 0
		print "Adding grasps to the new XML file..."
		for i in range(0, len(gmodel.grasps)):
			print str(i+1)+"/"+str(len(gmodel.grasps))
			try:

	 			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=gmodel.grasps[i],translate=True,forceclosure=True)
				#care-o-bot3.zae
				value = (finalconfig[0])[7:14]	
				value = OR_to_ROS(value)
				j = value

				if not (j[1]>=-1.15 and j[3]>=-1.15 and j[5]>=-1.25 and j[1]<=1 and j[3]<=1 and j[5]<=1 and j[2]>-0.5 and j[4]>-0.5 and j[6]>-0.5):
					continue

			   	f.write("<Grasp Index=\""+str(cont)+"\">\n")
                		value = ROS_to_script_server(value)
				f.write("<joint_values>"+str(value)+"</joint_values>\n")

				# [Valores relativos al palm_link]
			   	f.write("<GraspPose>\n")
				robot = env.GetRobots()[0]
				robot.GetController().Reset(0)
				robot.SetDOFValues(finalconfig[0])
				robot.SetTransform(finalconfig[1])
				env.UpdatePublishedBodies()
				index = (robot.GetLink("sdh_palm_link")).GetIndex()
				matrix = (robot.GetLinkTransformations())[index]
				t = translation_from_matrix(matrix)
				e = euler_from_matrix(matrix, axes='sxyz')
				f.write("<Translation>["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]</Translation>\n")
				f.write("<Rotation>["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]</Rotation>\n")
			   	f.write("</GraspPose>\n")

				(category, score) = getCategoryAndScore(matrix, finalconfig[0])
				offset = 0.1

				if category == "X":
					t[0] = t[0] - offset
				elif category == "-X": 
					t[0] = t[0] + offset
				elif category == "Y": 
					t[1] = t[1] - offset
				elif category == "-Y": 
					t[1] = t[1] + offset
				elif category == "Z":
					t[2] = t[2] - offset 
				elif category == "-Z":
					t[2] = t[2] + offset
				else: 
					t = t

		   		f.write("<PreGraspPose>\n")
				f.write("<Translation>["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]</Translation>\n")
				f.write("<Rotation>["+str(e[0])+", "+str(e[1])+", "+str(e[2])+"]</Rotation>\n")
		   		f.write("</PreGraspPose>\n")


				f.write("<category>"+category+"</category>\n")
				f.write("<score>"+str(score)+"</score>\n")

			   	f.write("</Grasp>\n")

				cont += 1
			
			except:
				continue


		f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n")
		f.write("</configuration>\n")
		f.write("</GraspList>")
		f.close()
		print "%d grasps have been added to the XML file..." %cont




# -----------------------------------------------------------------------------------------------
# XML file generator.
# -----------------------------------------------------------------------------------------------
def getCategoryAndScore(matrix, values):

	x = (matrix)[0][2]
	y = (matrix)[1][2]
	z = (matrix)[2][2]

	r1 = (values)[1]
	r2 = (values)[3]
	r3 = (values)[5]

	if (abs(r1)-abs(r2) > 0.6) or (abs(r1)-abs(r3) > 0.6) or (abs(r2)-abs(r3) > 0.6) or (abs(r1)-abs(r2) < -0.6) or (abs(r1)-abs(r3) < -0.6) or (abs(r2)-abs(r3) < -0.6):
		return "NONE", 0

	else:
		if x > 0.85:
			return "X", x

		elif x < -0.85:
			return "-X", x

		elif y > 0.85:
			return "Y", y

		elif y < -0.85:
			return "-Y", y

		elif z > 0.85:
			return "Z", z

		elif z < -0.85:
			return "-Z", z

		else:
			return "NONE", 0



# -----------------------------------------------------------------------------------------------
# Get grasps
# -----------------------------------------------------------------------------------------------
def getGrasps(file_name, all_grasps=False, msg=False):
	try:
		xmldoc = minidom.parse(file_name)  
	
		padres = ((xmldoc.firstChild)).getElementsByTagName('configuration')
		object_id = ((xmldoc.firstChild)).getElementsByTagName('object_id')[0].firstChild.nodeValue

		res = []
		for j in range(0, len(padres)):
			hijos = (((xmldoc.firstChild)).getElementsByTagName('configuration'))[j].getElementsByTagName('Grasp')
			grasps = []
			for i in range(0,len(hijos)):

				#score = (hijos[i].attributes["Index"]).value

				sconfiguration = ((hijos[i].getElementsByTagName('joint_values'))[0]).firstChild.nodeValue

				aux = ((hijos[i].getElementsByTagName('GraspPose'))[0])
				Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
				Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)
				palm_pose = PoseStamped()
				palm_pose.pose.position.x = float(Translation[0])
				palm_pose.pose.position.y = float(Translation[1])
				palm_pose.pose.position.z = float(Translation[2])
				Rotation =  quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
				palm_pose.pose.orientation.x = float(Rotation[0])
				palm_pose.pose.orientation.y = float(Rotation[1])
				palm_pose.pose.orientation.z = float(Rotation[2])
				palm_pose.pose.orientation.w = float(Rotation[3])




				aux = ((hijos[i].getElementsByTagName('PreGraspPose'))[0])
				Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
				Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)
				pre_grasp_pose = PoseStamped()
				pre_grasp_pose.pose.position.x = float(Translation[0])
				pre_grasp_pose.pose.position.y = float(Translation[1])
				pre_grasp_pose.pose.position.z = float(Translation[2])
				Rotation =  quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
				pre_grasp_pose.pose.orientation.x = float(Rotation[0])
				pre_grasp_pose.pose.orientation.y = float(Rotation[1])
				pre_grasp_pose.pose.orientation.z = float(Rotation[2])
				pre_grasp_pose.pose.orientation.w = float(Rotation[3])


				score = ((hijos[i].getElementsByTagName('score'))[0]).firstChild.nodeValue
				category = ((hijos[i].getElementsByTagName('category'))[0]).firstChild.nodeValue
	

				grasps.append(GraspConfig(object_id, score, category, sconfiguration, palm_pose, pre_grasp_pose))


			filtered_grasps, ALL_grasps = graspFilter(grasps)

			if all_grasps==True:
				#res.append(ALL_grasps)
				res = ALL_grasps
			else:
				#res.append(filtered_grasps)
				res = filtered_grasps


		if msg==False:
			return res					#returns a GraspConfig list (openrave viewer)
		else:
			return graspConfig_to_MSG(res)			#returns a msg list (server/service)
	except:
		return []

# -----------------------------------------------------------------------------------------------
# Get grasps by axis
# -----------------------------------------------------------------------------------------------
def getGraspsByAxis(grasps, axis=""):

	res = []
	if axis == "":
		return grasps

	if axis!="X" and axis!="-X" and axis!="Y" and axis!="-Y" and axis!="Z" and axis!="-Z":
		print "Incorrect axis name. The correct options are: X, -X, Y, -Y, Z, -Z"
		return []

	for i in range(0,len(grasps)):
		if grasps[i].category == axis:
			res.append(grasps[i])

	return res


# -----------------------------------------------------------------------------------------------
# Grasp function
# -----------------------------------------------------------------------------------------------
def GraspIt(grasp):
	sss.move("sdh", "cylopen")
	#pre-grasp to the palm_pose
	offset = 0.1
	
	#grasp to the palm_pose
	sss.move("sdh", [eval(grasp.sconfiguration)])

	
	
# -----------------------------------------------------------------------------------------------
# Remove grasps in wich the fingers are in a extrange position. (obsolet, we have filtered all the file)
# -----------------------------------------------------------------------------------------------
def grasp_filter(g):
	grasps = []
	for i in range(0,len(g)):
		j = eval(g[i].joint_values)
		if (j[1]>=-1.15 and j[3]>=-1.15 and j[5]>=-1.25 and j[1]<=1 and j[3]<=1 and j[5]<=1) and j[2]>-0.5 and j[4]>-0.5 and j[6]>-0.5:
			grasps.append(g[i])

	return grasps


# -----------------------------------------------------------------------------------------------
# Shows the grasps in OpenRAVE
# -----------------------------------------------------------------------------------------------
def showOR(env, grasps, gazebo=False, delay=0.5):

	env.SetViewer('qtcoin')
	time.sleep(1.0)


	manip = ((env.GetRobots()[0]).GetManipulator("arm"))
	robot = env.GetRobots()[0]
	with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):

		for i in range(0,len(grasps)):

		
			print 'pre-grasp %d/%d'%(i,len(grasps))

			robot.SetDOFValues(stringList_to_ORformat("[0.0,-0.9854,0.9472,-0.9854,0.9472,-0.9854,0.9472]"))
			Tgrasp = matrix_from_graspPose(grasps[i].pre_grasp)

			index = (robot.GetLink("sdh_palm_link")).GetIndex()
			matrix = (robot.GetLinkTransformations())[index]
			Tdelta = dot(Tgrasp,linalg.inv(matrix))
			for link in manip.GetChildLinks():
				link.SetTransform(dot(Tdelta,link.GetTransform()))

			env.UpdatePublishedBodies()
			raw_input("Go to grasp!")


			print 'grasp %d/%d'%(i,len(grasps))

			robot.SetDOFValues(stringList_to_ORformat(grasps[i].sconfiguration))
			Tgrasp = matrix_from_graspPose(grasps[i].palm_pose)

			index = (robot.GetLink("sdh_palm_link")).GetIndex()
			matrix = (robot.GetLinkTransformations())[index]
			Tdelta = dot(Tgrasp,linalg.inv(matrix))
			for link in manip.GetChildLinks():
				link.SetTransform(dot(Tdelta,link.GetTransform()))

			env.UpdatePublishedBodies()


			if gazebo==True:
				res = raw_input("Do you want to see this configuration in Gazebo? (y/n): ")
				if res=="y":
					GraspIt(grasps[i])

			if delay is None:
				raw_input('Next config.')
			elif delay > 0:
				time.sleep(delay)
			print "--------------------------"




###### [ GRASP FILTERS ] ##################################################
def graspFilter(grasps):

	grasps.sort(GraspConfig.mZ)

	AUX = [[],[],[],[],[],[]]

	for i in range(0,len(grasps)):
		r1 = eval(grasps[i].sconfiguration)[1]
		r2 = eval(grasps[i].sconfiguration)[3]
		r3 = eval(grasps[i].sconfiguration)[5]
		if (abs(r1)-abs(r2) > 0.6) or (abs(r1)-abs(r3) > 0.6) or (abs(r2)-abs(r3) > 0.6) or (abs(r1)-abs(r2) < -0.6) or (abs(r1)-abs(r3) < -0.6) or (abs(r2)-abs(r3) < -0.6):
			continue
		else:
			if grasps[i].category == "X":
				AUX[0].append(grasps[i])
			elif grasps[i].category == "-X":
				AUX[1].append(grasps[i])
			elif grasps[i].category == "Y":
				AUX[2].append(grasps[i])
			elif grasps[i].category == "-Y":
				AUX[3].append(grasps[i])
			elif grasps[i].category == "Z":
				AUX[4].append(grasps[i])
			elif grasps[i].category == "-Z":
				AUX[5].append(grasps[i])
			else:
				continue
	

	#All the grasps
	all_grasps = []
	all_grasps = AUX[0]+AUX[1]+AUX[2]+AUX[3]+AUX[4]+AUX[5]

	all_grasps.sort()

	#3 different grasps for each axys
	sol = []
	D = 3

	g1 = AUX[0]
	aux = len(g1)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g1[m])
	sol.append(g1[mm])
	sol.append(g1[media])
	sol.append(g1[M])
	sol.append(g1[MM])

	g2 = AUX[1]
	aux = len(g2)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g2[m])
	sol.append(g2[mm])
	sol.append(g2[media])
	sol.append(g2[M])
	sol.append(g2[MM])

	g3 = AUX[2]
	aux = len(g3)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g3[m])
	sol.append(g3[mm])
	sol.append(g3[media])
	sol.append(g3[M])
	sol.append(g3[MM])

	g4 = AUX[3]
	aux = len(g4)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g4[m])
	sol.append(g4[mm])
	sol.append(g4[media])
	sol.append(g4[M])
	sol.append(g4[MM])

	g5 = AUX[4]
	aux = len(g5)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g5[m])
	sol.append(g5[mm])
	sol.append(g5[media])
	sol.append(g5[M])
	sol.append(g5[MM])

	g6 = AUX[5]
	aux = len(g6)
	media = int(aux/2)
	decimo = int(media/D)
	M = media+decimo
	MM = int((M+media)/2)
	m = media-decimo
	mm = int((media+m)/2)
	if M==m==media:
		M = aux-1
		m = 0
	sol.append(g6[m])
	sol.append(g6[mm])
	sol.append(g6[media])
	sol.append(g6[M])
	sol.append(g6[MM])
	

	return sol, all_grasps



