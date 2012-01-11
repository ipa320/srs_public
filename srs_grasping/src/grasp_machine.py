#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')

import rospy
import smach
import smach_ros
import actionlib
import random
import tf
import time
import math

from simple_script_server import *
sss = simple_script_server()

from numpy import matrix
from kinematics_msgs.srv import *
from srs_grasping.msg import *
import grasping_functions
from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from srs_object_database.msg import *
from srs_object_database.srv import *

client = actionlib.SimpleActionClient('/grasp_server', GraspAction)


# ------------------------------------------------------------------------------------------
# STATES
# ------------------------------------------------------------------------------------------
# define state WAIT_SERVER
class Wait_grasp_server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
	print quaternion_from_euler(0.78, 0,0, axes='sxyz')
	"""
	print quaternion_from_euler(90,0,0, axes='sxyz')
	q = []
	q.append(0)
	q.append(0.85090352)
	q.append(0)
	q.append(0.52532199)
	e = euler_from_quaternion(q, axes='sxyz')
	print e 
	raw_input("Aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
	"""

    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_GRASP_SERVER')
	
	while not client.wait_for_server(rospy.Duration(1.0)):
        	rospy.logerr('Waiting for /grasp_server...')

	rospy.loginfo('/grasp_server found')
	return 'succeeded'



# define state READ_DB
class Read_DB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['object_id','pose_id'], output_keys=['grasp_configuration'])
	
    def execute(self, userdata):
        rospy.loginfo('Executing state READ_DB')

	goal = GraspGoal(object_id=userdata.object_id, pose_id=userdata.pose_id)
	client.send_goal(goal)
	client.wait_for_result()
	grasp_configuration = client.get_result().grasp_configuration


	if len(grasp_configuration)==0:
		rospy.logerr("No grasping configurations for this object.")
		return 'failed'
	else:
		rospy.loginfo("%d grasping configuration for this object." %len(grasp_configuration))	
		userdata.grasp_configuration = grasp_configuration
	        return 'succeeded'



# define state MOVE_ARM
class Move_arm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','retry','failed'], input_keys=['grasp_configuration', 'pose_id'], output_keys=['grasp_config'])
	self.listener = tf.TransformListener()
	self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
	self.counter = 0

    def get_joint_state(self, msg):
	global current_joint_configuration
	current_joint_configuration = list(msg.desired.positions)
	rospy.spin()

    def callIKSolver(self, current_pose, goal_pose):
	req = GetPositionIKRequest()
	req.ik_request.ik_link_name = "sdh_palm_link"
	req.ik_request.ik_seed_state.joint_state.position = current_pose
	req.ik_request.pose_stamped = goal_pose
	resp = self.iks(req)
	result = []
	for o in resp.solution.joint_state.position:
		result.append(o)
	return (result, resp.error_code)


    def matrix_from_graspPose(self,gp):
	q = []
	q.append(gp.pose.orientation.x)
	q.append(gp.pose.orientation.y)
	q.append(gp.pose.orientation.z)
	q.append(gp.pose.orientation.w)
	e = euler_from_quaternion(q, axes='sxyz')

	m = euler_matrix(e[0],e[1],e[2] ,axes='sxyz')
	m[0][3] = gp.pose.position.x
	m[1][3] = gp.pose.position.y
	m[2][3] = gp.pose.position.z

	m = matrix([[m[0][0], m[0][1], m[0][2], m[0][3]], [m[1][0], m[1][1], m[1][2], m[1][3]], [m[2][0], m[2][1], m[2][2], m[2][3]], [m[3][0], m[3][1], m[3][2], m[3][3]]])
	return m

    def getTransformation(self, object_pose):

	#OpenRAVE object position.
	OR_pose = PoseStamped()
	OR_pose.header.stamp = rospy.Time.now()
	OR_pose.header.frame_id = "/map"
	OR_pose.pose.position.x = 0
	OR_pose.pose.position.y = 0
	OR_pose.pose.position.z = 0
	OR_pose.pose.orientation.x = 0
	OR_pose.pose.orientation.y = 0
	OR_pose.pose.orientation.z = 0
	OR_pose.pose.orientation.w = 1

	inicial = self.matrix_from_graspPose(OR_pose)
	final = self.matrix_from_graspPose(object_pose)
	transformation = final*inicial.I
	return transformation


    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_ARM')
	listener = tf.TransformListener(True, rospy.Duration(10.0))

	if self.counter >= len(userdata.grasp_configuration):
		rospy.logerr('No good configurations for this object.')
		return 'failed'

        rospy.loginfo('Executing the grasp_configuration[%d]' %self.counter)

	#current_joint_configuration
	sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.get_joint_state)
	while sub.get_num_connections() == 0:
		time.sleep(0.3)
		continue
	"""
	#Detect
	self.srv_name_object_detection = '/object_detection/detect_object'
	detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
	req = DetectObjectsRequest()
	req.object_name.data = "detected_object"
	res = detector_service(req)

	for i in range(0,len(res.object_list.detections)):
		print str(i)+": "+res.object_list.detections[i].label

	index = 0
	while (index < 0) and (index >= res.object_list.detections):
		index = int(raw_input("Select object to grasp: "))

	Object_pose = listener.transformPose("/base_link", res.object_list.detections[index].pose)
	"""

	########### [chapuza hasta que la deteccion funcione]
	#x = ((float(int(random.uniform(-1,0)*100)))/100)
	#y = ((float(int(random.uniform(-0.5,0.5)*100)))/100)

	Object_pose = PoseStamped()
	Object_pose.header.stamp = rospy.Time.now()
	Object_pose.header.frame_id = "/map"
	Object_pose.pose.position.x = -0.6
	Object_pose.pose.position.y = 0
	Object_pose.pose.position.z = 0.55 
	Object_pose.pose.orientation.x = 0
	Object_pose.pose.orientation.y = 0
	Object_pose.pose.orientation.z = 0
	Object_pose.pose.orientation.w = 1
	Object_pose = listener.transformPose("/base_link", Object_pose)
	print Object_pose
	########### [/chapuza hasta que la deteccion funcione]

	pre_grasp_pose = userdata.grasp_configuration[self.counter].pre_grasp
	grasp_pose = userdata.grasp_configuration[self.counter].palm_pose
	tf_matrix = self.getTransformation(Object_pose)

	pre_trans = tf_matrix*self.matrix_from_graspPose(pre_grasp_pose)
	grasp_trans = tf_matrix*self.matrix_from_graspPose(grasp_pose)

	t = translation_from_matrix(pre_trans)
	q = quaternion_from_matrix(pre_trans)
	tg = translation_from_matrix(grasp_trans)
	qg = quaternion_from_matrix(grasp_trans)

	pre = PoseStamped()
	pre.header.stamp = rospy.Time.now()
	pre.header.frame_id = "/base_link"
	pre.pose.position.x = t[0] 
	pre.pose.position.y = t[1] 
	pre.pose.position.z = t[2] 
	pre.pose.orientation.x = q[0]
	pre.pose.orientation.y = q[1]
	pre.pose.orientation.z = q[2]
	pre.pose.orientation.w = q[3]

	g = PoseStamped()
	g.header.stamp = rospy.Time.now()
	g.header.frame_id = "/base_link"
	g.pose.position.x = tg[0] 
	g.pose.position.y = tg[1] 
	g.pose.position.z = tg[2] 
	g.pose.orientation.x = qg[0]
	g.pose.orientation.y = qg[1]
	g.pose.orientation.z = qg[2]
	g.pose.orientation.w = qg[3]

	
	offset_x = (g.pose.position.x - pre.pose.position.x)/2
	offset_y = (g.pose.position.y - pre.pose.position.y)/2
	offset_z = (g.pose.position.z - pre.pose.position.z)/2
	
	pre.pose.position.x += offset_x
	pre.pose.position.y += offset_y
	pre.pose.position.z += offset_z
	g.pose.position.x += offset_x
	g.pose.position.y += offset_y
	g.pose.position.z += offset_z
	

	#Sometimes the IKSolver does not find solutions for a valid position. For that, we call it 20 times.
	sol = False
	for i in range(0,10):
		(pre_grasp_conf, error_code) = self.callIKSolver(current_joint_configuration, pre)		
		if(error_code.val == error_code.SUCCESS):
			sol = True
			break
	if not sol:
		rospy.logerr("Ik pre_grasp FAILED")
		self.counter += 1
		return 'retry'


	sol = False
	for i in range(0,10):
		(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, g)		
		if(error_code.val == error_code.SUCCESS):
			sol = True
			break

	if not sol:
		rospy.logerr("Ik grasp FAILED")
		self.counter += 1
		return 'retry'


	res = raw_input("Execute this grasp? (y/n): ")
	if res != "y":
		self.counter += 1
		return 'retry'

	# execute grasp
	sss.say(["I am grasping the object now."], False)
	raw_input("ARM: Pregrasp...")
	handle_arm = sss.move("arm", [pre_grasp_conf], False)
	sss.move("sdh", "cylopen")
	handle_arm.wait()
	raw_input("ARM: Grasp...")
	sss.move("arm", [grasp_conf])


	userdata.grasp_config = userdata.grasp_configuration[self.counter]
	return 'succeeded'



# define state MOVE_HAND
class Move_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['grasp_config'])
	
    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_HAND')
	raw_input("Execute grasp:")
	print list(userdata.grasp_config.sconfiguration.points[0].positions)
	print "--------------------"
	#values = grasping_functions.ROS_to_script_server()
	sss.move("sdh", [list(userdata.grasp_config.sconfiguration.points[0].positions)])
	return 'succeeded'




# ------------------------------------------------------------------------------------------
# MAIN FUNCTION 
# ------------------------------------------------------------------------------------------
def main(object_id, pose_id):

    #sss.move("arm", "home")
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['succeeded','failed'])


    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('WAIT_GRASP_SERVER', Wait_grasp_server(), transitions={'succeeded':'READ_DB'})
        smach.StateMachine.add('READ_DB', Read_DB(), transitions={'succeeded':'GRASP_OBJECT', 'failed': 'failed'})

    	sm_sub = smach.StateMachine(outcomes=['succeeded','failed','retry'])
	with sm_sub:
		smach.StateMachine.add('MOVE_ARM', Move_arm(), transitions={'succeeded':'MOVE_HAND', 'retry': 'retry', 'failed': 'failed'})
        	smach.StateMachine.add('MOVE_HAND', Move_hand())


        smach.StateMachine.add('GRASP_OBJECT', sm_sub, transitions={'succeeded': 'succeeded', 'retry': 'GRASP_OBJECT', 'failed': 'failed'})


    sm_top.userdata.object_id = object_id		
    sm_top.userdata.pose_id = pose_id   
    sm_sub.userdata = sm_top.userdata
  

    #smach_viewer = smach_ros.IntrospectionServer('GRASP_SMACH', sm_top, 'GRASP_SMACH')
    #smach_viewer.start()


    # Execute SMACH plan
    outcome = sm_top.execute()
	
    #rospy.spin()
    #smach_viewer.stop()

    return outcome



# ------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('grasp_state_machine')
    main(1, "Y")	#object_id = Milk
