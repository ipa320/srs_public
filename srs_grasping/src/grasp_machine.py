#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')

import rospy
import smach
import smach_ros
import actionlib
import random

from simple_script_server import *
sss = simple_script_server()

from kinematics_msgs.srv import *
from srs_grasping.msg import *
client = actionlib.SimpleActionClient('/grasp_server', GraspAction)


# ------------------------------------------------------------------------------------------
# STATES
# ------------------------------------------------------------------------------------------
# define state WAIT_SERVER
class Wait_grasp_server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

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
        smach.State.__init__(self, outcomes=['succeeded','retry','failed'], input_keys=['grasp_configuration'])
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

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_ARM')

	if self.counter >= len(userdata.grasp_configuration):
		rospy.logerr('No good configurations for this object.')
		return 'failed'

        rospy.loginfo('Executing the grasp_configuration[%d]' %self.counter)

	#current_joint_configuration
	sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, self.get_joint_state)
	while sub.get_num_connections() == 0:
		time.sleep(0.3)
		continue
	
	pre_grasp_pose = userdata.grasp_configuration[self.counter].pre_grasp
	grasp_pose = userdata.grasp_configuration[self.counter].palm_pose

	#TODO: Detect an object and transform its pose into base_link coordinates system.
	#TODO: Transform palm_pose/pre_grasp in object coordinates system.

	#Sometimes the IKSolver does not find solutions for a valid position. For that, we call it 20 times.
	sol = False
	for i in range(0,20):
		(pre_grasp_conf, error_code) = self.callIKSolver(current_joint_configuration, pre_grasp_pose)		
		if(error_code.val == error_code.SUCCESS):
			sol = True
			continue
	if not sol:
		rospy.logerr("Ik pre_grasp FAILED")
		self.counter += 1
		return 'retry'
		

	sol = False
	for i in range(0,20):
		(grasp_conf, error_code) = self.callIKSolver(pre_grasp_conf, grasp_pose)		
		if(error_code.val == error_code.SUCCESS):
			sol = True
			continue

	if not sol:
		rospy.logerr("Ik grasp FAILED")
		self.counter += 1
		return 'retry'


	# execute grasp
	sss.say(["I am grasping the object now."], False)
	handle_arm = sss.move("arm", [pre_grasp_conf , grasp_conf], False)
	sss.move("sdh", "cylopen")
	handle_arm.wait()

	return 'succeeded'



# define state MOVE_HAND
class Move_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
	
    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_HAND')

	#values = srs_grasping.ROS_to_script_server(list(res[self.retries].sconfiguration.points[0].positions))
	values = grasping_functions.ROS_to_script_server(list(res[self.retries].sconfiguration.points[0].positions))
	sss.move("sdh", [values])
	return 'succeeded'




# ------------------------------------------------------------------------------------------
# MAIN FUNCTION 
# ------------------------------------------------------------------------------------------
def main(object_id, pose_id):


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
  

    smach_viewer = smach_ros.IntrospectionServer('GRASP_SMACH', sm_top, 'GRASP_SMACH')
    smach_viewer.start()


    # Execute SMACH plan
    outcome = sm_top.execute()
	
    rospy.spin()
    smach_viewer.stop()

    return outcome



# ------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('grasp_state_machine')
    main(1, "X")	#object_id = Milk, pose_id = X
