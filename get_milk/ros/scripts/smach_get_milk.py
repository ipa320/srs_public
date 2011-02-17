#!/usr/bin/python

import time

import roslib; roslib.load_manifest('cob_script_server')
import rospy

import smach
import smach_ros

from simple_script_server import script

import tf
from geometry_msgs.msg import *


class INIT(smach.State):

	def __init__(self):

		smach.State.__init__(self, outcomes=['goto'])
		
	def execute(self, userdata):

		# initialize components (not needed for simulation)
		sss.init("torso")
		sss.init("tray")
		sss.init("arm")
		sss.init("sdh")
		sss.init("head")

		sss.init("base")

		return 'goto'


class INIT_POS(smach.State):

	def __init__(self):

		smach.State.__init__(self, outcomes=['goto'])

	def execute(self, userdata):

		# move to initial positions
		handle_torso = sss.move("torso", "home", False)
		handle_tray = sss.move("tray", "down", False)
		handle_arm = sss.move("arm", "folded", False)
		handle_sdh = sss.move("sdh", "cylclosed", False)
		handle_head = sss.move("head", "back", False)

		# wait for initial movements to finish
		handle_torso.wait()
		handle_tray.wait()
		handle_arm.wait()
		handle_sdh.wait()
		handle_head.wait()

		# move base to initial position
		handle_base = sss.move("base", "table", False)

		# wait for base to reach initial position
		handle_base.wait()

		return 'goto'


class GRASP(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['goto'])

	def execute(self, userdata):

		listener = tf.TransformListener(True, rospy.Duration(10.0))

		# move arm to pregrasp position
		handle_arm = sss.move("arm", "pregrasp", False)
		handle_sdh = sss.move("sdh", "cylopen", False)

		# wait for arm movements to finish
		handle_arm.wait()
		handle_sdh.wait()

		# calculate tranformations, cup coordinates needed for coordination
		cup = PointStamped()
		cup.header.stamp = rospy.Time.now()
		cup.header.frame_id = "/map"
		cup.point.x = -0.3
		cup.point.y = -1.39
		cup.point.z = 0.8
		sss.sleep(2)

		if not sss.parse:
			cup = listener.transformPoint('/arm_7_link', cup)

		# grasp milk box
		sss.move("arm",'pregrasp_2')
		sss.move_cart_rel("arm",[[0.0, 0.0, 0.2], [0, 0, 0]])
		sss.move("sdh", "cylclosed")

		# move milk box
		sss.move_cart_rel("arm",[[0.0, 0.4, 0.0], [0, 0, 0]])

		return 'goto'


def main():

	rospy.init_node('smach_script_GET_MILK')

	# Create a SMACH state machine
	GET_MILK = smach.StateMachine(outcomes=['DONE'])

	# Open the container
	with GET_MILK:

		# Add states to the container
		smach.StateMachine.add('Initiate', INIT(), transitions={'goto':'INIT_POS'})
		smach.StateMachine.add('Initial_Positions', INIT_POS(), transitions={'goto':'GRASP'})
		smash.StateMachine.add('Grasp_Milk', GRASP(), transitions={'goto':'DONE'})

	sis = smach_ros.IntrospectionServer('GET_MILK', GET_MILK, 'GET_MILK')
	sis.start()

	# Execute SMACH plan
	outcome = GET_MILK.execute()

	rospy.spin()
	sis.stop()

if __name__ == "__main__":
	main()
