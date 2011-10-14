#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')

import rospy
import smach
import smach_ros

import actionlib
import grasping_functions
from srs_grasping.msg import *


from simple_script_server import *
sss = simple_script_server()


package_path = roslib.packages.get_pkg_dir('srs_grasping')
client = actionlib.SimpleActionClient('/grasp_server', GraspAction)

# define state WAIT_SERVER
class Wait_server(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], output_keys=['object_id','pose_id'])


    def execute(self, userdata):
        rospy.loginfo('Executing state WAIT_SERVER')
	
	if not client.wait_for_server(rospy.Duration(120)):
        	rospy.logerr('Time expired: /grasp_server not found')
		return 'failed'
	else:
        	rospy.loginfo('/grasp_server found')
		return 'succeeded'



# define state READ_DB
class Read_DB(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['object_id','pose_id'], output_keys=['grasp_configuration','index'])

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
		userdata.index = 0
	        return 'succeeded'
	

# define state GRASP_OBJECT
class Grasp_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','retry','failed'], input_keys=['grasp_configuration','grasp_index_in'], output_keys=['grasp_index_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP_OBJECT')

	if userdata.grasp_index_in >= len(userdata.grasp_configuration):
		rospy.logerr('No good configurations for this object.')
		return 'failed'



        rospy.loginfo('Executing the grasp_configuration[%d]' %userdata.grasp_index_in)
	# --- PREGRASP ---
	#move_arm_result = IK solver pre-grasp position feedback from IPA (boolean?)
	move_arm_result = False		#for example

	while not move_arm_result:
		if userdata.grasp_index_in < len(userdata.grasp_configuration):
			userdata.grasp_index_out = userdata.grasp_index_in + 1
			rospy.logerr('This configuration does not work.')
			return 'retry'

	# --- GRASP ---
	# IK solver grasp position feedback from IPA
	sss.move("sdh", [list(userdata.grasp_configuration[userdata.grasp_index_in].sconfiguration.points[0].positions)])


	rospy.logerr('Founded a good configuration for this object.')
	return 'succeeded'



def main():
    rospy.init_node('grasp_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('WAIT_SERVER', Wait_server(), transitions={'succeeded':'READ_DB', 'failed': 'failed'})
        smach.StateMachine.add('READ_DB', Read_DB(), transitions={'succeeded':'GRASP_OBJECT', 'failed': 'failed'})
        smach.StateMachine.add('GRASP_OBJECT', Grasp_object(), transitions={'succeeded':'succeeded', 'retry': 'GRASP_OBJECT', 'failed': 'failed'}, remapping={'grasp_index_in': 'index', 'grasp_index_out': 'index'})

	

    # Who gives this info and how?
    sm.userdata.object_id = 0	# Milk box (for example)
    sm.userdata.pose_id = "X"   # X axis (for example)


    smach_viewer = smach_ros.IntrospectionServer('GRASP_SMACH', sm, 'GRASP_SMACH')
    smach_viewer.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    smach_viewer.stop()


if __name__ == '__main__':
    main()
