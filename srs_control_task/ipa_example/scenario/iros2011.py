#!/usr/bin/python

import roslib; roslib.load_manifest('cob_iros2011')
import rospy

import smach
import smach_ros

# generic states
from generic_basic_states import *
from generic_navigation_states import *
from generic_manipulation_states import *
from generic_perception_states import *

# generic state machines
from generic_state_machines import *

# scenario specific states
from iros2011_states import *

# main
def main():
	rospy.init_node('iros2011')


	# create a SMACH state machine
	SM = smach.StateMachine(outcomes=['succeeded','failed'])
	#SM.userdata.task_active = False

	# open the container
	with SM:

		# add states to the container
		smach.StateMachine.add('INITIALIZE', initialize(),
			transitions={'succeeded':'WAIT_FOR_TASK', 'failed':'failed'})
		
		smach.StateMachine.add('WAIT_FOR_TASK', wait_for_task(['quit', 'initialize', 'serve_drink']),
			transitions={'quit':'succeeded', 'initialize':'INITIALIZE', 'serve_drink':'MOVE_TO_ORDER_POSITION'})

		smach.StateMachine.add('MOVE_TO_ORDER_POSITION', approach_pose("order"),
			transitions={'succeeded':'GET_ORDER', 'failed':'failed'})

		smach.StateMachine.add('GET_ORDER', get_order(),
			transitions={'succeeded':'SELECT_LOCATION', 'no_order':'WAIT_FOR_TASK', 'failed':'failed'})

		smach.StateMachine.add('SELECT_LOCATION', select_location(),
			transitions={'shelf':'MOVE_TO_GRASP_POSITION', 'table':'MOVE_TO_GRASP_POSITION', 'cabinet':'MOVE_TO_CABINET', 'object_not_known':'WAIT_FOR_TASK'})

		smach.StateMachine.add('MOVE_TO_CABINET', approach_pose("cabinet_door"),
			transitions={'succeeded':'SM_OPEN_DOOR', 'failed':'failed'})
		
		smach.StateMachine.add('SM_OPEN_DOOR', sm_open_door(),
			transitions={'succeeded':'MOVE_TO_GRASP_POSITION', 'door_not_opened':'WAIT_FOR_TASK', 'failed':'failed'})

		smach.StateMachine.add('MOVE_TO_GRASP_POSITION', approach_pose(),
			transitions={'succeeded':'SM_PICK_OBJECT', 'failed':'failed'})

		smach.StateMachine.add('SM_PICK_OBJECT', sm_pick_object(),
			transitions={'succeeded':'PUT_OBJECT_ON_TRAY', 'object_not_picked':'WAIT_FOR_TASK', 'failed':'failed'})

		smach.StateMachine.add('PUT_OBJECT_ON_TRAY', put_object_on_tray(),
				transitions={'succeeded':'MOVE_TO_DELIVER_POSITION', 'failed':'failed'})

		smach.StateMachine.add('MOVE_TO_DELIVER_POSITION', approach_pose("order"),
			transitions={'succeeded':'DELIVER_OBJECT', 'failed':'failed'})

		smach.StateMachine.add('DELIVER_OBJECT', deliver_object(),
			transitions={'succeeded':'SAY_GOODBYE', 'failed':'failed'})

		smach.StateMachine.add('SAY_GOODBYE', say_goodbye(),
			transitions={'succeeded':'WAIT_FOR_TASK', 'failed':'failed'})

	# Start SMACH viewer
	smach_viewer = smach_ros.IntrospectionServer('IROS2011', SM, 'IROS2011')
	smach_viewer.start()

	SM.execute()

	# stop SMACH viewer
	rospy.spin()
	# smach_thread.stop()
	smach_viewer.stop()

if __name__ == '__main__':
	main()
