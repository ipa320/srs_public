#!/usr/bin/python

import roslib; roslib.load_manifest('cob_srs')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *
import time


class GetMilk(script):

	def Initialize(self):

		# assign listener
		self.listener = tf.TransformListener(True, rospy.Duration(10.0))

		# initialize components (not needed for simulation)
		self.sss.init("torso")
		self.sss.init("tray")
		self.sss.init("arm")
		self.sss.init("sdh")
		self.sss.init("head")

		self.sss.init("base")

		# move to initial positions
		handle_torso = self.sss.move("torso", "back", False)
		handle_tray = self.sss.move("tray", "down", False)
		handle_arm = self.sss.move("arm", "folded", False)
		handle_sdh = self.sss.move("sdh", "cylclosed", False)
		handle_head = self.sss.move("head", "back", False)

		# wait for initial movements to finish
		handle_torso.wait()
		handle_tray.wait()
		handle_arm.wait()
		handle_sdh.wait()
		handle_head.wait()

		# localize the robot with rviz
		if not self.sss.parse:
			print "Please localize the robot with rviz"
		self.sss.wait_for_input()

		# move base to initial position
		self.handle_base = self.sss.move("base", "kitchen")

	def Run(self):

		# move arm to pregrasp position
		self.sss.move("arm", "pregrasp")
		self.sss.move("sdh", "cylopen")

		# detect milk
		self.sss.detect('milk')
		milk_box = PoseStamped()
		milk_box = self.sss.get_object_pose('milk')

		# calculate tranformations
		self.sss.sleep(2)
		if not self.sss.parse:
			milk_box = self.listener.transformPose('/arm_7_link', milk_box)

		# grasp milk box
		self.sss.move_cart_rel("arm", [[0.05, milk_box.pose.position.y, milk_box.pose.position.z-0.40], [0.0, 0.0, 0.0]])
		self.sss.move_cart_rel("arm", [[0.0, 0.0, 0.2], [0.0, 0.0, 0.0]])
		self.sss.move("sdh", "milk_box")

		# move milk box above tray
		handle_tray = self.sss.move("tray", "up", False)
		handle_torso = self.sss.move("torso", "home", False)
		self.sss.move_cart_rel("arm", [[0.2, -0.2, 0.0], [0.0, 0.0, 0.0]])
		self.sss.move("arm", "grasp-to-tray")

		# wait for tray and torso
		handle_tray.wait()
		handle_torso.wait()

		# put milk box onto tray
		self.sss.move_cart_rel("arm", [[-0.02, 0.0, 0.0], [0, 0, 0]])
		self.sss.move("sdh", "cylopen")

		# move arm to folded position
		self.sss.move_cart_rel("arm", [[0.0, 0.0, -0.12], [0, 0, 0]])
		self.sss.move("sdh","cylclosed")
		self.sss.move("arm", "tray-to-folded")

		# deliver milk box
		self.sss.move("base", "order")
		self.sss.move("torso", "bow")


if __name__ == "__main__":
	SCRIPT = GetMilk()
	SCRIPT.Start()
