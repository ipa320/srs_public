#!/usr/bin/python

import roslib; roslib.load_manifest('srs_demo_gesture')
import rospy

from simple_script_server import script

import tf
from geometry_msgs.msg import *
import time


class DemoGesture(script):

	def Initialize(self):

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
		self.handle_base = self.sss.move("base", "home")

	def Run(self):
		# move base for testing
		self.sss.wait_for_input()
		self.sss.move("base", "left")
		self.sss.wait_for_input()
		self.sss.move("base", "right")


if __name__ == "__main__":
	SCRIPT = DemoGesture()
	SCRIPT.Start()
