#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import grasping_functions
from shared_state_information import *


from simple_script_server import *
sss = simple_script_server()

from srs_grasping.srv import *

class select_grasp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['object'],
            output_keys=['grasp_configuration'])
        
        #default grasp categorisation
        #self.grasp_configuration = ""
        self.object_id=1

    def execute(self, userdata):
        
        global listener
        try:
            # transform object_pose into base_link
            object_pose_in = userdata.object.pose
            print object_pose_in
            object_pose_in.header.stamp = listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
            object_pose_bl = listener.transformPose("/base_link", object_pose_in)
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'
        
        get_grasps_from_position = rospy.ServiceProxy('get_grasps_from_position', GetGraspsFromPosition)
        req = GetGraspsFromPositionRequest(self.object_id, object_pose_bl.pose)
        grasp_configuration = (get_grasps_from_position(req)).grasp_configuration
       
        userdata.grasp_configuration = grasp_configuration[1]
        
        return 'succeeded'



class grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','not_completed','retry' ,'failed', 'preempted'], input_keys=['grasp_configuration'])
	
    def get_joint_state(self, msg):
	global current_joint_configuration
	current_joint_configuration = list(msg.desired.positions)
	rospy.spin()

    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP')
	
	#Open SDH at the pre-grasp position -----------------------------------------------
	sss.move("sdh", "cylopen")

	#Get the current arm joint states.
	sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
	while sub.get_num_connections() == 0:
		time.sleep(0.3)
		continue

	#Move to grasp position with SDH open ---------------------------------------------
	#pregrasp
	pre_grasp_stamped = PoseStamped();
	pre_grasp_stamped.header.frame_id = "/base_link";
	pre_grasp_stamped.pose = userdata.grasp_configuration.pre_grasp;

	#grasp
	grasp_stamped = PoseStamped();
	grasp_stamped.header.frame_id = "/base_link";
	grasp_stamped.pose = userdata.grasp_configuration.grasp;

	#offset
	"""
	offset_x = 0#(userdata.grasp_configuration.grasp.position.x - userdata.grasp_configuration.pre_grasp.position.x)/3
	offset_y = 0#(userdata.grasp_configuration.grasp.position.y - userdata.grasp_configuration.pre_grasp.position.y)/3
	offset_z = 0#(userdata.grasp_configuration.grasp.position.z - userdata.grasp_configuration.pre_grasp.position.z)/3

	pre_grasp_stamped.pose.position.x += offset_x
	pre_grasp_stamped.pose.position.y += offset_y
	pre_grasp_stamped.pose.position.z -= offset_z
	grasp_stamped.pose.position.x += offset_x
	grasp_stamped.pose.position.y += offset_y
	grasp_stamped.pose.position.z -= offset_z
	"""


	sol = False
	for i in range(0,10):
		(pre_grasp_conf, error_code) = grasping_functions.callIKSolver(current_joint_configuration, pre_grasp_stamped)
		if(error_code.val == error_code.SUCCESS):	
			for j in range(0,10):	
				(grasp_conf, error_code) = grasping_functions.callIKSolver(pre_grasp_conf, grasp_stamped)
				sol = True
				break
		if sol:
			break;


	if not sol:
		return 'failed';
	else:
		arm_handle = sss.move("arm", [pre_grasp_conf], False)
		arm_handle.wait();
		rospy.sleep(2);
		arm_handle = sss.move("arm", [grasp_conf], False)
		arm_handle.wait();
		rospy.sleep(2);

		#Close SDH based on the grasp configuration to grasp. 
		arm_handle = sss.move("sdh", [list(userdata.grasp_configuration.sdh_joint_values)], False)
		arm_handle.wait();
		rospy.sleep(2);

		#TODO: Confirm the grasp based on force feedback
		successful_grasp = grasping_functions.sdh_tactil_sensor_result();

		if successful_grasp:
			return 'succeeded'
		else:
			#TODO: Regrasp (close MORE the fingers)
			regrasp = list(userdata.grasp_configuration.sdh_joint_values)
			regrasp[2] -= 0.1
			regrasp[4] -= 0.1
			regrasp[6] -= 0.1
			arm_handle = sss.move("sdh", [regrasp], False)
			arm_handle.wait();

			successful_regrasp =  grasping_functions.sdh_tactil_sensor_result();
			if successful_regrasp:
				return 'succeeded'
			else:
				return 'failed'
			#return 'failed'
