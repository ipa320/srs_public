#!/usr/bin/env python
#################################################################################
# SRS project - translate_server.py                                             #
# Place : Manufacturing Engineering Centre, Cardiff University                  #
# Author : Damien LEBON, Sylvain Chardonneau & Gwendal LOUBES                   #
# Date : August 2011                                                            #
#-------------------------------------------------------------------------------#
# Description : This is a simple service which use a service-client protocol to #
# translate the message from the user to make the command understandable by the #
# robot. So this server is called by the decision making which sends the command#
# make by user then it returns the low level message corresponding              #
#################################################################################


import roslib; roslib.load_manifest('srs_control_task')
import rospy

from srs_control_task.srv import *



####Translation of a high level message####
def handle_translate(req):    
    print "Returning [%s]"%(req.highLevelMess)
    return req.highLevelMess

def translate_server():
    #Launch server and wait an input from a client
    rospy.init_node('translate_server')
    s = rospy.Service('translate', translation, handle_translate)
    print "Ready to translate."
    rospy.spin()
    return s

if __name__ == "__main__":
    translate_server()
