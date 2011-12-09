#!/usr/bin/env python
#################################################################
# \note randomly generate historical data(just mimic the DEM to generate historical data)
# \input(from DEM) action sequence for task
# \output(from DEM to learning) action sequence for task
# \feedback(from learning to DEM) non
#   Project name: srs learning service for choosing priority 
# \author
#   Tao Cao, email:itaocao@gmail.com
#
# \date Date of creation: Dec 2011
#################################################################

import roslib; roslib.load_manifest('srs_likelihood_calculation')
import rospy
from std_msgs.msg import String

from random import choice

"""
tasks=['get milk table1','get book table2','bring a cup of tea','open the door','get cup table1','help me walk around','cook paster','hoover the living room','get milk table2','get book table1','get cup table2','get milk fridge','get cup table3','help me walk around']

intervals= [2,5,3,4,7,8,9,10,15,17,19,20]

"""

#the candidates locations/patent object for milkbox

foo = ['table2', 'table1','fridge']

def historical_data_publisher():
    pub = rospy.Publisher('historical_data', String)
    rospy.init_node('DEM_talker')
    while not rospy.is_shutdown():
        #str = "%s %s"%(rospy.get_time(), choice(tasks))
		#location= choice(foo)
		action="move(base,%s)"%choice(foo)#randomly chose locations
		actions=[action]
		actions.append('detect(milkbox)')
		action="move(base,%s)"%choice(foo)#randomly chose locations
		actions.append(action)
		actions.append('detect(milkbox)')
		actions.append('grasp(Milk box)')
		actions.append('place_on_tray(Milk box)')		
		
		#str='The action sequence is: '.join(actions)
		str=', '.join(actions)
		#str=actions
		print str	
		#print'\n'
        	#rospy.loginfo(str)
        	pub.publish(String(str))
		#interval_between_tasks=choice(intervals)
		#print "the random interval between tasks is %d" %interval_between_tasks
        	rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        historical_data_publisher()
    except rospy.ROSInterruptException: pass
