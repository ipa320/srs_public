#!/usr/bin/env python

from xml.dom import minidom
import unicodedata

import roslib; roslib.load_manifest('ROB')
import rospy
import simple_script_server

from numpy import *
from openravepy import *
from tf.transformations import *
from trajectory_msgs.msg import *




class GraspConfig():

	def __init__(self, Index, joint_values, GraspPose, MinDist, Volume):
		self.Index = Index
		self.joint_values = joint_values
		self.GraspPose = GraspPose
		self.MinDist = MinDist
		self.Volume = Volume;

	
	def __cmp__(self,other):
		if self.Volume == other.Volume:
			return 0
		elif self.Volume > other.Volume:
			return -1
		else:
			return 0
	
	"""
	def __cmp__(self,other):
		if eval(self.GraspPose.Translation)[2] == eval(other.GraspPose.Translation)[2]:
			return 0
		elif eval(self.GraspPose.Translation)[2] > eval(other.GraspPose.Translation)[2]:
			return -1
		else:
			return 0
	"""






class GraspPose():
	def __init__(self, Translation, Rotation):
		self.Translation = Translation
		self.Rotation = Rotation



# --------------------------------------------------------------------------------------------------------------------
# XML file generator.
# --------------------------------------------------------------------------------------------------------------------
def generaFicheroXML(all_grasps, targetName, realTargetName, envName, gmodel):

	# ------------------------ 
	# targetName.xml (Only for remote_lab tasks)
	# ------------------------ 

	img_name = raw_input("Put the image name for this configuration: \n");
	try:
		fmain = open("../DB/"+targetName+".xml",'r+');
		xmldoc = minidom.parse("../DB/"+targetName+".xml");
		confs = (((xmldoc.firstChild)).getElementsByTagName('configuration')).length;
		lineas = fmain.readlines();
		lineas[len(lineas)-1]="<FileName id=\""+img_name+"\">\n"+"<configuration>"+str(confs)+"</configuration>\n"+"</FileName>\n"
		lineas.append("</"+targetName+">\n");

		fmain = open("../DB/"+targetName+".xml",'w');
		for i in range(0, len(lineas)):
			fmain.write(str(lineas[i]));

	except:
		fmain = open("../DB/"+targetName+".xml",'w');
		fmain.write("<?xml version=\"1.0\" ?>\n");
		fmain.write("<"+targetName+">\n");
		fmain.write("<FileName id=\"" + img_name + "\">\n");
		fmain.write("<configuration>0</configuration>\n");
		fmain.write("</FileName>\n");
		fmain.write("</"+targetName+">\n");
		
	fmain.close()
	print "Created entry in the file <%s.xml>" %targetName;



	
	# ------------------------ 
	# <targetName_all_grasps>.xml
	# ------------------------ 

	f_name = targetName + "_all_grasps.xml"



	try:
		f = open('../DB/'+f_name,'r')
		addConfiguration(f_name, gmodel)
	except:
		f = open('../DB/'+f_name,'w')
		f.write("<?xml version=\"1.0\" ?>\n");
		f.write("<GraspList>\n");
		f.write("<Object>"+targetName+"</Object>\n");
		f.write("<HAND>SDH</HAND>\n");
		f.write("<joint_names>[sdh_knuckle_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint, sdh_thumb_2_joint, sdh_thumb_3_joint]</joint_names>\n");
		f.write("<configuration id=\"0\">\n")


		cont = 0
		print "Adding grasps to the new XML file..."
		for i in range(0, len(gmodel.grasps)):
			try:
	 			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=gmodel.grasps[i],translate=True,forceclosure=True)
			   	f.write("<Grasp Index=\""+str(cont)+"\">\n")

				value = OR_to_ROS(finalconfig[0])[0];
				f.write("<joint_values>"+str(value)+"</joint_values>\n");

			   	f.write("<GraspPose>\n");
				t = translation_from_matrix(finalconfig[1])
				f.write("<Translation>["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]</Translation>\n");
				q = quaternion_from_matrix(finalconfig[1])
				f.write("<Rotation>["+str(q[0])+", "+str(q[1])+", "+str(q[2])+", "+str(q[3])+"]</Rotation>\n");
			   	f.write("</GraspPose>\n")

			   	f.write("<MinDist>"+str(mindist)+"</MinDist>\n")
			   	f.write("<Volume>"+str(volume)+"</Volume>\n")

			   	f.write("</Grasp>\n")

				cont += 1
			
			except:
				continue




		f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n");
		f.write("</configuration>\n");
		f.write("</GraspList>");
		f.close()
		print "Se anyadieron %d al fichero XML..." %cont;
		print "%d grasps have been added to the XML file..." %cont;





# --------------------------------------------------------------------------------------------------------------------
# XML file depurator.
# --------------------------------------------------------------------------------------------------------------------
def generaFicheroDepurado(targetName, grasps):
	
	# ------------------------ 
	# <targetName_D>.xml
	# ------------------------ 

	f_name = targetName + "_D.xml"



	try:
		f = open('../DB/'+f_name,'r')
		addDepurateConfiguration(f_name, grasps)
	except:
		f = open('../DB/'+f_name,'w')
		f.write("<?xml version=\"1.0\" ?>\n");
		f.write("<GraspList>\n");
		f.write("<Object>"+targetName+"</Object>\n");
		f.write("<HAND>SDH</HAND>\n");
		f.write("<joint_names>[sdh_knuckle_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint, sdh_thumb_2_joint, sdh_thumb_3_joint]</joint_names>\n");
		f.write("<configuration id=\"0\">\n")


		cont = 0
		print "Adding grasps to the NEW depurated file..."
		for i in range(0, len(grasps)):
			try:
			   	f.write("<Grasp Index=\""+str(i)+"\">\n")

				f.write("<joint_values>"+grasps[i].joint_values+"</joint_values>\n");

			   	f.write("<GraspPose>\n");
				f.write("<Translation>"+grasps[i].GraspPose.Translation+"</Translation>\n");
				f.write("<Rotation>"+grasps[i].GraspPose.Rotation+"</Rotation>\n");
			   	f.write("</GraspPose>\n")

			   	f.write("<MinDist>"+grasps[i].MinDist+"</MinDist>\n")
			   	f.write("<Volume>"+grasps[i].Volume+"</Volume>\n")

			   	f.write("</Grasp>\n")

				cont += 1
			
			except:
				continue




		f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n");
		f.write("</configuration>\n");
		f.write("</GraspList>");
		f.close()
		print "%d have been added to the depurated file..." %cont;








# --------------------------------------------------------------------------------------------------------------------
# OpenRAVE/Gazebo to Gazebo/OpenRAVE transformation joint values.
# --------------------------------------------------------------------------------------------------------------------
def OR_to_ROS(OR):

	pi = math.pi
	ROS = [[OR[0]+pi/3, OR[3]-pi/2,OR[4] ,OR[1]-pi/2,OR[2], OR[5]-pi/2, OR[6]]]
	return ROS

def ROS_to_OR(OR):

	OR = eval(OR)
	for i in range (0,len(OR)):
		OR[i] = float(OR[i])
	pi = math.pi
	ROS = [OR[0]-pi/3, OR[3]+pi/2, OR[4], OR[1]+pi/2, OR[2], OR[5]+pi/2, OR[6]]
	return ROS


# --------------------------------------------------------------------------------------------------------------------
# Get grasps
# --------------------------------------------------------------------------------------------------------------------
def getGrasps(file_name):

	file_name = "../DB/"+file_name

	xmldoc = minidom.parse(file_name)  
	padres = ((xmldoc.firstChild)).getElementsByTagName('configuration');

	res = []
	for j in range(0, len(padres)):
		hijos = (((xmldoc.firstChild)).getElementsByTagName('configuration'))[j].getElementsByTagName('Grasp');
		grasps = []
		for i in range(0,len(hijos)):


			Index = hijos[i].getAttribute('Index');

			joint_values = ((hijos[i].getElementsByTagName('joint_values'))[0]).firstChild.nodeValue;

			aux = ((hijos[i].getElementsByTagName('GraspPose'))[0]);
			Translation = (aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue;
			Rotation = (aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue;
			gp = GraspPose(Translation, Rotation);

			MinDist = ((hijos[i].getElementsByTagName('MinDist'))[0]).firstChild.nodeValue;
			Volume = ((hijos[i].getElementsByTagName('Volume'))[0]).firstChild.nodeValue;

			grasps.append(GraspConfig(Index, joint_values, gp, MinDist, Volume));
			grasps.sort();
			
		res.append(grasps);
	return res;



# --------------------------------------------------------------------------------------------------------------------
# Shows the grasps in OpenRAVE
# --------------------------------------------------------------------------------------------------------------------
def showOR(env, grasps, delay=0.5, depurador=False):
	g = []

	if depurador==True:
		print "Write <save> to save the current configuration."


	for i in range(0,len(grasps)):

		print 'grasp %d/%d'%(i,len(grasps))
		env.GetRobots()[0].GetController().Reset(0)
		env.GetRobots()[0].SetDOFValues(ROS_to_OR(grasps[i].joint_values))	
		env.GetRobots()[0].SetTransform(matrix_from_graspPose(grasps[i].GraspPose))
		env.UpdatePublishedBodies()


		if depurador==True:
			res = raw_input("?: " )
			if res=="save":
				g.append(grasps[i]);
		else:
			if delay is None:
				raw_input('Next config.')
			elif delay > 0:
				time.sleep(delay)

	return g;


# --------------------------------------------------------------------------------------------------------------------
# Returns a homogeneus matrix.
# --------------------------------------------------------------------------------------------------------------------
def matrix_from_graspPose(gp):

	translation = eval(gp.Translation)
	rotation = eval(gp.Rotation)

	matrix = quaternion_matrix(rotation)
	matrix[0][3] = translation[0]
	matrix[1][3] = translation[1]
	matrix[2][3] = translation[2]

	return matrix;
	

# --------------------------------------------------------------------------------------------------------------------
# Add a new configuration to the XML file.
# --------------------------------------------------------------------------------------------------------------------
def addConfiguration(file_name, gmodel):

	# ------------------------ 
	# <targetName_all_grasps>.xml
	# ------------------------ 


	f = open('../DB/'+file_name,'r+')
	lines = f.readlines();
	f = open('../DB/'+file_name,'w')
	xmldoc = minidom.parse("../DB/"+file_name[0:-15]+".xml");
	confs = (((xmldoc.firstChild)).getElementsByTagName('configuration')).length;
	lines[len(lines)-1] = "<configuration id=\""+str(confs-1)+"\">\n"
	for i in range(0, len(lines)):
		f.write(str(lines[i]));
	
	cont = 0
	print "Adding a new configuration to the old XML file..."
	for i in range(0, len(gmodel.grasps)):
		try:
 			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=gmodel.grasps[i],translate=True,forceclosure=True)
		   	f.write("<Grasp Index=\""+str(cont)+"\">\n")

			value = OR_to_ROS(finalconfig[0])[0];
			f.write("<joint_values>"+str(value)+"</joint_values>\n");

		   	f.write("<GraspPose>\n");
			t = translation_from_matrix(finalconfig[1])
			f.write("<Translation>["+str(t[0])+", "+str(t[1])+", "+str(t[2])+"]</Translation>\n");
			q = quaternion_from_matrix(finalconfig[1])
			f.write("<Rotation>["+str(q[0])+", "+str(q[1])+", "+str(q[2])+", "+str(q[3])+"]</Rotation>\n");
		   	f.write("</GraspPose>\n")

		   	f.write("<MinDist>"+str(mindist)+"</MinDist>\n")
		   	f.write("<Volume>"+str(volume)+"</Volume>\n")

		   	f.write("</Grasp>\n")

			cont += 1
			
		except:
			continue




	f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n");
	f.write("</configuration>\n");
	f.write("</GraspList>");
	f.close()
	print "%d have been added to the XML file..." %cont;



# --------------------------------------------------------------------------------------------------------------------
# Add a new configuration to the XML depurated file.
# --------------------------------------------------------------------------------------------------------------------
def addDepurateConfiguration(file_name, grasps):

	# ------------------------ 
	# <targetName_D>.xml
	# ------------------------ 


	f = open('../DB/'+file_name,'r+')
	lines = f.readlines();
	f = open('../DB/'+file_name,'w')
	xmldoc = minidom.parse("../DB/"+file_name[0:-6]+".xml");
	confs = (((xmldoc.firstChild)).getElementsByTagName('configuration')).length;
	lines[len(lines)-1] = "<configuration id=\""+str(confs-1)+"\">\n"
	for i in range(0, len(lines)):
		f.write(str(lines[i]));
	
	cont = 0
	print "Adding a new configuration to the old depurated XML file..."
	for i in range(0, len(grasps)):
		try:
		   	f.write("<Grasp Index=\""+str(cont)+"\">\n")


			f.write("<joint_values>"+grasps[i].joint_values+"</joint_values>\n");

		   	f.write("<GraspPose>\n");
			f.write("<Translation>"+grasps[i].GraspPose.Translation+"</Translation>\n");
			f.write("<Rotation>"+grasps[i].GraspPose.Rotation+"</Rotation>\n");
		   	f.write("</GraspPose>\n")

		   	f.write("<MinDist>"+grasps[i].MinDist+"</MinDist>\n")
		   	f.write("<Volume>"+grasps[i].Volume+"</Volume>\n")

		   	f.write("</Grasp>\n")

			cont += 1
			
		except:
			continue




	f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n");
	f.write("</configuration>\n");
	f.write("</GraspList>");
	f.close()
	print "%d have been added to the XML depurated file..." %cont;



# --------------------------------------------------------------------------------------------------------------------
# Grasp function
# -------------------------------------------------------------------------------------------------------------------
def graspIt(objectID, poseID):
	#WARNING! The move functions sometimes returns an error when the configuration works.

	#file_name = '../DB/'+objectID+"_D"		#Refined file.
	file_name = '../DB/'+objectID+"_all_grasps.xml"	#Non refined file.

	GRASPS = getGrasps(file_name)
	grasps = GRASPS[poseID];

	
	rospy.init_node("grasping_node")
	sss = simple_script_server.simple_script_server()

	
	fail = False;
	for i in range(0,len(grasps)):
		values = eval(grasps[i].joint_values)

		for j in range(0,len(values)):
			values[j] = float(values[j]);
		
			if j==0:
				if values[j]<0:
					values[j] = 0;

		res = sss.move("sdh", [values])
		if res.get_error_code() == 0:
			print "Good grasp."
			break;
		else:
			print "Wrong grasp."



