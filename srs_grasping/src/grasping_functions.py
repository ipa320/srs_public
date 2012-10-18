#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')

import graspingutils
import databaseutils
import openraveutils

graspingutils = graspingutils.graspingutils(simulation=True);
databaseutils = databaseutils.databaseutils(graspingutils);
openraveutils = openraveutils.openraveutils(databaseutils);
