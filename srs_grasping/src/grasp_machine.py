#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')

import rospy
import smach
import smach_ros
import actionlib


from simple_script_server import *
sss = simple_script_server()

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
	self.counter = 0

    def get_joint_state(msg):
	global current_joint_configuration
	current_joint_configuration = list(msg.desired.positions)
	rospy.spin()

    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_ARM')

	if self.counter >= len(userdata.grasp_configuration):
		rospy.logerr('No good configurations for this object.')
		return 'failed'

        rospy.loginfo('Executing the grasp_configuration[%d]' %self.counter)

	#sub = rospy.Subscriber("/sdh_controller/state", JointTrajectoryControllerState, get_joint_state)
	#while sub.get_num_connections() == 0:
	#	time.sleep(0.3)
	#	continue

	#(conf1, pre_grasp_result) = ik_solver_function(current_joint_configuration, userdata.grasp_configuration[self.counter].pre_grasp)
	#(conf2, grasp_result) = ik_solver_function(conf1, userdata.grasp_configuration[self.counter].palm_pose)

	#------
	pre_grasp_result = True	
	grasp_result = True
	#------


	if (pre_grasp_result == False) or (grasp_result == False):
		rospy.logerr('This configuration does not work.')
		self.counter += 1
		return 'retry'


	rospy.loginfo('This configuration has worked.')
	#move_arm_function(conf1)
	#sss.move("sdh", "cylopen")
	#move_arm_function(conf2)
	return 'succeeded'



# define state MOVE_ARM
class Move_hand(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
	
    def execute(self, userdata):
        rospy.loginfo('Executing state MOVE_HAND')
	#sss.move("sdh", [list(userdata.grasp_configuration[userdata.grasp_index_in].sconfiguration.points[0].positions)])
	#if move sdh fails: return 'failed'
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
    main(0, "X")	#object_id = Milk, pose_id = X
