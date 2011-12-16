#!/usr/bin/env python
#################################################################
# \note likelihood client
# \input(from DEM ) command and candidates
# \output(from DEM to learning) command and candidates
# \feedback(from learning to DEM) candidate and correspomding likelihood
#   Project name: srs learning service for choosing priority 
# \author
#   Tao Cao, email:itaocao@gmail.com
#
# \date Date of creation: Dec 2011
#################################################################

import roslib; roslib.load_manifest('srs_likelihood_calculation')

import sys

import rospy
from srs_likelihood_calculation.srv import *

def likelihood_client(x, y):
    rospy.wait_for_service('likelihood')
    try:
        likelihood = rospy.ServiceProxy('likelihood', Likelihood)
        resp1 = likelihood(x, y)
        print "DEM wants to know likelihood for %s  from %s"%(x,y)
	return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [command candidates]"%sys.argv[0]

if __name__ == "__main__":
  
    #input the consulting command
    x=raw_input("Please enter the consulting command end with Enter: \n")
    #input the cadidates
    y=raw_input("Please enter the candidates for the consulting command:the format is\n    cadidate1 candidate2 candidate3 ...,\nend with Enter: \n")
    
    print likelihood_client(x, y)

