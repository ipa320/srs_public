#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.msg import Command

def commander():
   	
	pub = rospy.Publisher('command', Command)
   	rospy.init_node('commander')
	rospy.sleep(1.0)

	while not rospy.is_shutdown():
		cmd = Command()
		cmd.grasp = 3
		cmd.rb_x = 2.1
		cmd.rb_y = 1.45
		cmd.rb_theta = 1.2
		cmd.obj_x = 1.3
		cmd.obj_y = 1.4
		cmd.obj_theta = 1.5
		print cmd
		rospy.loginfo(cmd)
		pub.publish(cmd)
		rospy.sleep(1.0)
	


if __name__ == '__main__':
	try:
		commander()
	except rospy.ROSInterruptException: pass

