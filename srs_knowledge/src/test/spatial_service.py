#!/usr/bin/env python
import roslib;
roslib.load_manifest('srs_knowledge')
import sys
import rospy

from srs_knowledge.srv import *
from srs_knowledge.msg import *

import numpy as np

def transform_2d_point(theta, x, y):
    nx, ny = x, y
    R = np.array([np.cos(theta), -np.sin(theta), np.sin(theta), np.cos(theta)])
    R = R.reshape(2,2)
    pos = np.array([x,y])
    pos = pos.reshape(2,1)
    new_pos = R.dot(pos)
    return new_pos

if __name__=='__main__':
    print 'service ---- for spatial relation calculation'
    print transform_2d_point(np.pi, -1, 1)
    
