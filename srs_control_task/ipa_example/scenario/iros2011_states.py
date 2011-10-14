#!/usr/bin/python

import roslib; roslib.load_manifest('cob_iros2011')
import rospy

import smach
import smach_ros


class select_location(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['shelf','table','cabinet','object_not_known'], 
			input_keys=['object_name'], 
			output_keys=['base_pose'])
		
		self.table_objects = ['fanta','coke']
		self.shelf_objects = ['chocolate','chips']
		self.cabinet_objects = ['milk']

	def execute(self, userdata):
		#TODO classify order: where to grasp from
		print userdata.object_name
		if self.table_objects.count(userdata.object_name) != 0: # check if object is in table_objects list
			userdata.base_pose = "table"
			return 'table'
		elif self.shelf_objects.count(userdata.object_name) != 0: # check if object is in shelf_objects list
			userdata.base_pose = "shelf"
			return 'shelf'
		elif self.cabinet_objects.count(userdata.object_name) != 0: # check if object is in cabinet_objects list
			userdata.base_pose = "cabinet"
			return 'cabinet'
		else:
			sss.say(["Sorry, I don't know where to search for the object."])
			rospy.logerror("object not known")
			return 'object_not_known'

class say_goodbye(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['succeeded', 'failed'])

	def execute(self, userdata):
		#TODO implement goodbye
		return 'succeeded'
