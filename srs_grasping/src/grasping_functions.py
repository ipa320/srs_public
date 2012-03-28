#!/usr/bin/env python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import openravepy
import os
import time
import tf

from srs_grasping.srv import *
#from srs_object_database.srv import *

from tf.transformations import *
from numpy import *
from xml.dom import minidom

from srs_msgs.msg import *
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from kinematics_msgs.srv import *
from schunk_sdh.msg import *

#---------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------
#---------------------------------------------------------------------------------------------------------------
def init_env(object_id):

	robotName = roslib.packages.get_pkg_dir("srs_grasping")+"/robots/care-o-bot3.zae"
	env = openravepy.Environment()

	try:
    		robot = env.ReadRobotXMLFile(robotName)
		robot.SetActiveManipulator("arm")		#care-o-bot3.zae
		env.AddRobot(robot)
	except:
		rospy.logerr("The robot file %s does not exists.", robotName)
		return (-1,-1);

	get_mesh = rospy.ServiceProxy('/get_model_mesh', GetMesh)

	try:
		resp = get_mesh(model_ids=[object_id])
	except rospy.ServiceException, e:
		rospy.logerr("Service did not process request: %s", str(e))
		return (-1,-1);

	try:

		mesh_file = "/tmp/mesh.iv"
		f = open(mesh_file, 'w')
		res = f.write(resp.msg[0].data)
		f.close()

		target = env.ReadKinBodyXMLFile(mesh_file)
		env.AddKinBody(target)
		os.remove(mesh_file)
	except:
		rospy.logerr("The mesh data does not exist or does not work correctly.")
		os.remove(mesh_file)
		return (-1,-1);


	gmodel = openravepy.databases.grasping.GraspingModel(robot=robot,target=target)

	return (gmodel, env);


def generate_grasp_file(object_id, gmodel, env):

	generate_grasps(gmodel);

	f_name = "/tmp/"+str(object_id)+".xml"

	f = open(f_name,'w')
	f.write("<?xml version=\"1.0\" ?>\n")
	f.write("<GraspList>\n")
	f.write("<object_id>"+str(object_id)+"</object_id>\n")
	f.write("<hand_type>SDH</hand_type>\n")
	f.write("<joint_names>[sdh_knuckle_joint, sdh_thumb_2_joint, sdh_thumb_3_joint, sdh_finger_12_joint, sdh_finger_13_joint, sdh_finger_22_joint, sdh_finger_23_joint]</joint_names>\n")
	f.write("<grasp_reference_link>sdh_palm_link</grasp_reference_link>\n")

	cont = 0
	print "Adding grasps to the new XML file..."
	for i in range(0, len(gmodel.grasps)):
		try:
 			contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp=gmodel.grasps[i],translate=True,forceclosure=True)
			OR = (finalconfig[0])[7:14]	#care-o-bot3.zae
			values = [OR[2], OR[3], OR[4], OR[0], OR[1], OR[5], OR[6]]	 #values to script_server
			j = values

			if not (j[1]>=-1.15 and j[3]>=-1.15 and j[5]>=-1.25 and j[1]<=1 and j[3]<=1 and j[5]<=1 and j[2]>-0.5 and j[4]>-0.5 and j[6]>-0.5):
				continue

		   	f.write("<Grasp Index=\""+str(cont)+"\">\n")
			value = [values[0], values[5], values[6], values[1], values[2], values[3], values[4]]
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

			category = get_category(matrix, finalconfig[0])
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
		   	f.write("</Grasp>\n")

			cont += 1
		except:
			continue


	f.write("<NumberOfGrasps>"+str(cont)+"</NumberOfGrasps>\n")
	f.write("</GraspList>")
	f.close()

	f = open(f_name, 'r')
	res = f.read()
	f.close()

	os.remove(f_name)
	print "%d grasps have been added to the XML file." %cont
	return res;


def generate_grasps(gmodel):

	if not gmodel.load():
		rospy.loginfo("GENERATING GRASPS...")
		gmodel.autogenerate()
		rospy.loginfo("GENERATING GRASPS HAS FINISHED.")


def insert_grasps_in_DB(object_id, grasp_file):

	insert_obj = rospy.ServiceProxy('/insert_object_service', InsertObject)
	try:
		resp = insert_obj(model_id=object_id, data_grasp=grasp_file)
	except rospy.ServiceException, e:
	  	rospy.logerr("Service did not process request: %s", str(e))
		return -1


def generator(object_id):

	(gmodel, env) = init_env(object_id);
	if (gmodel == -1 or env == -1):
		return -1;
	grasp_file = generate_grasp_file(object_id, gmodel, env);
	insert_grasps_in_DB(object_id, grasp_file);
	return 0;


def get_category(matrix, values):

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
			return "X"

		elif x < -0.85:
			return "-X"

		elif y > 0.85:
			return "Y"

		elif y < -0.85:
			return "-Y"

		elif z > 0.85:
			return "Z"

		elif z < -0.85:
			return "-Z"

		else:
			return "NONE"


def read_grasps_from_DB(object_id):

	server_result = GetGraspConfigurationsResponse();
	get_grasp = rospy.ServiceProxy('/get_model_grasp', GetGrasp);

	resp = get_grasp(model_ids=[object_id]);

	if len(resp.msg) == 0:
		rospy.loginfo("No grasping data for this object.");	
		return -1;

	try:
		grasp_file = "/tmp/grasp.xml";
		f = open(grasp_file, 'w');
		f.write(resp.msg[0].bs);
		f.close();

		GRASPS = get_grasps_from_file(grasp_file);
		if GRASPS == -1:
			rospy.logerr("ERROS reading the grasp file");
			return -1;

		os.remove(grasp_file);

		rospy.loginfo(str(len(GRASPS))+" grasping configuration for this object.");	

		server_result.grasp_configuration = GRASPS;
		return server_result;

	except rospy.ServiceException, e:
		rospy.logerr("Service did not process request: %s", str(e));
		rospy.logerr("No grasping data for this object.");
		return -1;


def get_grasps_from_file(file_name):

	try:
		xmldoc = minidom.parse(file_name)  

		object_id = int(((xmldoc.firstChild)).getElementsByTagName('object_id')[0].firstChild.nodeValue)
		res = []

		hijos = (xmldoc.firstChild).getElementsByTagName('Grasp')
		grasps = []
		for i in range(0,len(hijos)):

			sdh_joint_values = ((hijos[i].getElementsByTagName('joint_values'))[0]).firstChild.nodeValue

			aux = (hijos[i].getElementsByTagName('GraspPose'))[0]
			Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
			Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)
			grasp = PoseStamped()
			grasp.header.frame_id = "/base_link"; #"/sdh_palm_link" ???
			grasp.pose.position.x = float(Translation[0])
			grasp.pose.position.y = float(Translation[1])
			grasp.pose.position.z = float(Translation[2])
			Rotation =  quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
			grasp.pose.orientation.x = float(Rotation[0])
			grasp.pose.orientation.y = float(Rotation[1])
			grasp.pose.orientation.z = float(Rotation[2])
			grasp.pose.orientation.w = float(Rotation[3])

			aux = (hijos[i].getElementsByTagName('PreGraspPose'))[0]
			Translation = eval((aux.getElementsByTagName('Translation')[0]).firstChild.nodeValue)
			Rotation = eval((aux.getElementsByTagName('Rotation')[0]).firstChild.nodeValue)
			pre_grasp = PoseStamped()
			pre_grasp.header.frame_id = "/base_link"; #"/sdh_palm_link" ???
			pre_grasp.pose.position.x = float(Translation[0])
			pre_grasp.pose.position.y = float(Translation[1])
			pre_grasp.pose.position.z = float(Translation[2])
			Rotation =  quaternion_from_euler(Rotation[0], Rotation[1], Rotation[2], axes='sxyz')
			pre_grasp.pose.orientation.x = float(Rotation[0])
			pre_grasp.pose.orientation.y = float(Rotation[1])
			pre_grasp.pose.orientation.z = float(Rotation[2])
			pre_grasp.pose.orientation.w = float(Rotation[3])

			category = ((hijos[i].getElementsByTagName('category'))[0]).firstChild.nodeValue


			GC = GraspConfiguration()

			GC.object_id = object_id
			GC.hand_type = "SDH"
			GC.sdh_joint_values = eval(sdh_joint_values)
			GC.target_link = "/sdh_palm_link"
			GC.pre_grasp = pre_grasp
			GC.grasp = grasp
			GC.category= str(category)

			grasps.append(GC)

		return grasps
	except:
		print "There are not generated files for this object."
		return -1


def get_grasp_category(pre, g):

	x = pre.x - g.x;
	y = pre.y - g.y;
	z = pre.z - g.z;

	if (x > 0.08):
		category = "FRONT";
	elif (x < -0.08):
		category = "BACK";
	elif (y > 0.08):
		category = "SIDE";
	elif (y < -0.08):
		category = "-SIDE";
	elif (z > 0.08):
		category = "TOP";
	elif (z < -0.08):
		category = "DOWN";
	else:
		category = "UNKNOWN";

	return category;


def callIKSolver(current_pose, goal_pose):

	iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)


	req = GetPositionIKRequest();
	req.ik_request.ik_link_name = "sdh_palm_link";
	req.ik_request.ik_seed_state.joint_state.position = current_pose;
	req.ik_request.pose_stamped = goal_pose;
	resp = iks(req);
	result = [];
	for o in resp.solution.joint_state.position:
		result.append(o);
	return (result, resp.error_code);


def matrix_from_pose(gp): 
	return numpy.matrix(array_from_pose(gp))


def array_from_pose(gp): 

	q = []
	q.append(gp.orientation.x)
	q.append(gp.orientation.y)
	q.append(gp.orientation.z)
	q.append(gp.orientation.w)
	e = euler_from_quaternion(q, axes='sxyz')

	matrix = euler_matrix(e[0],e[1],e[2] ,axes='sxyz')
	matrix[0][3] = gp.position.x
	matrix[1][3] = gp.position.y
	matrix[2][3] = gp.position.z
	return matrix


def rotation_matrix(obj):

	e = euler_from_quaternion([obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w],axes='sxyz');
	rotacion =  euler_matrix(e[0],e[1],e[2], axes='sxyz');
	rotacion[0,3] = obj.position.x;
	rotacion[1,3] = obj.position.y;
	rotacion[2,3] = obj.position.z;
	return rotacion;


def COB_to_OR(values):

	res = [0 for i in range(28)]	
	values = [values[5], values[6], values[0], values[3], values[4], values[1], values[2]]

	for i in range (0,len(values)):
		res[i+7] = float(values[i])
	return eval(str(res))


def SCRIPTSERVER_to_GAZEBO(values):	 #non-defined
	res = [0 for i in range(28)]	
	values = [values[5], values[6], values[0], values[3], values[4], values[1], values[2]]

	for i in range (0,len(values)):
		res[i+7] = float(values[i])
	return eval(str(res))


def GAZEBO_to_SCRIPTSERVER(values):	#non-defined
	res = [0 for i in range(28)]	
	values = [values[5], values[6], values[0], values[3], values[4], values[1], values[2]]

	for i in range (0,len(values)):
		res[i+7] = float(values[i])
	return eval(str(res))


def show_all_grasps(object_id, grasps):		#Group of grasps (grasp of GraspConfiguration)

	(gmodel, env) = init_env(object_id);

	env.SetViewer('qtcoin')
	time.sleep(1.0)

	manip = ((env.GetRobots()[0]).GetManipulator("arm"))
	robot = env.GetRobots()[0]

	with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):

		for i in range(0,len(grasps)):
			print 'grasp %d/%d'%(i,len(grasps))

			dof_values = COB_to_OR(grasps[i].sdh_joint_values);
			robot.SetDOFValues(dof_values);
			Tgrasp = array_from_pose(grasps[i].grasp.pose)
			index = (robot.GetLink("sdh_palm_link")).GetIndex()
			matrix = (robot.GetLinkTransformations())[index]
			Tdelta = dot(Tgrasp,linalg.inv(matrix))
			for link in manip.GetChildLinks():
				link.SetTransform(dot(Tdelta,link.GetTransform()))
			env.UpdatePublishedBodies()

			raw_input('Next config.')


def grasp_view(env, object_id, grasp, object_pose):	#Individual grasp (grasp of GraspSubConfiguration)

	env = SetRobot(env);
	env = SetTarget(env, object_id);

	m = matrix_from_pose(grasp.grasp);
	rot = numpy.matrix(rotation_matrix(object_pose));
	sol = rot.I * m;
	t = translation_from_matrix(sol)
	q = quaternion_from_matrix(sol)
	grasp.grasp.position.x = t[0]
	grasp.grasp.position.y = t[1]
	grasp.grasp.position.z = t[2] 
	grasp.grasp.orientation.x = q[0]
	grasp.grasp.orientation.y = q[1]
	grasp.grasp.orientation.z = q[2]
	grasp.grasp.orientation.w = q[3]


	manip = ((env.GetRobots()[0]).GetManipulator("arm"))
	robot = env.GetRobots()[0]
	with openravepy.databases.grasping.GraspingModel.GripperVisibility(manip):
		dof_values = COB_to_OR(grasp.sdh_joint_values)
		robot.SetDOFValues(dof_values)
		Tgrasp = array_from_pose(grasp.grasp)
		index = (robot.GetLink("sdh_palm_link")).GetIndex()
		matrix = (robot.GetLinkTransformations())[index]
		Tdelta = dot(Tgrasp,linalg.inv(matrix))
		for link in manip.GetChildLinks():
			link.SetTransform(dot(Tdelta,link.GetTransform()))
		env.UpdatePublishedBodies()
		rospy.sleep(5);
		env.Reset();


def init_simulator():
	robotName = roslib.packages.get_pkg_dir("srs_grasping")+"/robots/care-o-bot3.zae"
	env = openravepy.Environment()
	env.SetViewer('qtcoin');
	time.sleep(1.0)
	return env


def SetTarget(env, object_id):
	get_mesh = rospy.ServiceProxy('/get_model_mesh', GetMesh)
	try:
		resp = get_mesh(model_ids=[object_id])
	except rospy.ServiceException, e:
		rospy.logerr("Service did not process request: %s", str(e))
		return (-1,-1);
	try:
		mesh_file = "/tmp/mesh.iv"
		f = open(mesh_file, 'w')
		res = f.write(resp.msg[0].data)
		f.close()
		target = env.ReadKinBodyXMLFile(mesh_file)
		env.AddKinBody(target)
		os.remove(mesh_file)
	except:
		rospy.logerr("The mesh data does not exist or does not work correctly.")
		os.remove(mesh_file)
		return (-1,-1);
	return env


def SetRobot(env):
	robotName = roslib.packages.get_pkg_dir("srs_grasping")+"/robots/care-o-bot3.zae"
	try:
    		robot = env.ReadRobotXMLFile(robotName)
		robot.SetActiveManipulator("arm")		#care-o-bot3.zae
		env.AddRobot(robot)
	except:
		rospy.logerr("The robot file %s does not exists.", robotName)
		return -1
	return env


def sdh_tactil_sensor_result():
	rospy.loginfo("Reading SDH tactil sensors...");
	sub = rospy.Subscriber("/sdh_controller/tactile_data", TactileSensor, sdh_tactil_sensor_result_callback);
	while sub.get_num_connections() == 0:
		time.sleep(0.3);
		continue;
	rospy.loginfo("SDH tactil sensors has been readed.");
	return object_grasped;


def sdh_tactil_sensor_result_callback(msg):
	global object_grasped;
	tactile_matrix = msg.tactile_matrix
	
	count = 0;
	count_fingers = 0;
	for i in range(0, len(tactile_matrix)):
		tactile_array = tactile_matrix[i].tactile_array;
		for j in range(0, len(tactile_array)):
			if tactile_array[j] > 0:
				count+=1;
			if count > 10:
				count_fingers+=1;	#The finger touch the object
				break;		

	if count_fingers <4:
		object_grasped = False;
	else:
		object_grasped = True;
