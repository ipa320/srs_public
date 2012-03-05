#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import grasping_functions

from simple_script_server import *
sss = simple_script_server()

class grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['grasp_configuration'])
	
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
	sol = False
	for i in range(0,10):
		(grasp_conf, error_code) = grasping_functions.callIKSolver(current_joint_configuration, userdata.grasp_configuration.grasp)
		if(error_code.val == error_code.SUCCESS):		
			print str(i)+": IK solution found"
			sol = True
			break
	if not sol:
		return 'failed';
	else:
		sss.move("arm", [grasp_conf])

		#Close SDH based on the grasp configuration to grasp. 
		sss.move("sdh", [userdata.grasp_configuration.sdh_joint_values])

		#TODO: Confirm the grasp based on force feedback
		successful_grasp = grasping_functions.sdh_tactil_sensor_result();

		if successful_grasp:
			return 'succeeded'
		else:
			return 'failed'

