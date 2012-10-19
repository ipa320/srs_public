#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('srs_assisted_arm_navigation')
import rospy
import actionlib

from srs_assisted_arm_navigation_msgs.msg import *
from srs_interaction_primitives.srv import AddObject
from srs_interaction_primitives.srv import AddUnknownObject
from srs_env_model_percp.srv import EstimateBBAlt
from srs_interaction_primitives.srv import RemovePrimitive
from srs_interaction_primitives.srv import SetAllowObjectInteraction
from srs_interaction_primitives.msg import PoseType
from std_msgs.msg import ColorRGBA


def remove_im():
    
    remove_object = rospy.ServiceProxy('/interaction_primitives/remove_primitive', RemovePrimitive)
        
    try:
    
        remove_object(name='unknown_object')
            
        return True
      
    except Exception, e:
      
        rospy.logerr('Cannot remove IM object from the scene, error: %s',str(e))
            
        return False
            
            
def set_interaction(object_name,allow_int):
      
      
   rospy.wait_for_service('/interaction_primitives/set_allow_object_interaction')
   
   int_allow_srv =  rospy.ServiceProxy('/interaction_primitives/set_allow_object_interaction', SetAllowObjectInteraction)
   
   try:
       
       res = int_allow_srv(name = object_name,
                           allow = allow_int)
    
    
   except Exception, e:
    
    rospy.logerr('Cannot set interaction mode for object %s, error: %s',object_name,str(e))  

def bb_est_feedback(feedback):
    
    global object_added
    
    rospy.loginfo('Feedback received')
    
    print feedback
    
    est = rospy.ServiceProxy('/bb_estimator/estimate_bb_alt',EstimateBBAlt)
      
    header=rospy.Header() 
   
    header.frame_id = '/map'
    header.stamp = feedback.timestamp
    
    rospy.loginfo('Calling bb est srv')
    
      
    try:
          
      est_result = est(header=header,
                       p1=feedback.p1,
                       p2=feedback.p2,
                       mode=1)
          
    except Exception, e:
          
      rospy.logerr("Error on calling service: %s",str(e))
      return
  
    rospy.loginfo('BB estimation performed!')
  
    # /interaction_primitives/set_pregrasp_position'
    # /interaction_primitives/remove_primitive
 
    print "estimation result:"
    print est_result

    if object_added == True:
        
        if remove_im() == True:
            
            object_added = False
            
 
    if object_added == False:
 
        add_object = rospy.ServiceProxy('/interaction_primitives/add_unknown_object', AddUnknownObject)
        rospy.loginfo('Calling %s service','/interaction_primitives/add_unknown_object')
 
        #add_object = rospy.ServiceProxy('/interaction_primitives/add_object', AddObject)
        #rospy.loginfo('Calling %s service','/interaction_primitives/add_object')
    
        # color of the bounding box
        color = ColorRGBA()
        color.r = 1
        color.g = 1
        color.b = 0
        color.a = 1
           
        try:
          
            #===================================================================
            # add_object(frame_id = '/map',
            #           name = 'unknown_object',
            #           description = 'Unknown object to grasp',
            #           pose = est_result.pose,
            #           bounding_box_lwh = est_result.bounding_box_lwh,
            #           color = color,
            #           use_material = False)
            #===================================================================
            
            add_object(frame_id='/map',
                       name='unknown_object',
                       description='Unknown object to grasp',
                       pose_type= PoseType.POSE_BASE,
                       pose = est_result.pose,
                       scale = est_result.bounding_box_lwh)
            
            object_added = True
            
            rospy.loginfo('IM added')
          
          
        except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
          
        # allow interaction for this object  
        set_interaction('unknown_object',True)
      
    else:
        
        rospy.logerr('Could not add new IM - old one was not removed!');
        
    return


def main():
    
  global object_added
  
  object_added = False
    
  rospy.init_node('grasping_action_test')
  rospy.loginfo("Node for testing manual grasping actionlib server")
 
  client = actionlib.SimpleActionClient('/but_arm_manip/manual_bb_estimation_action',ManualBBEstimationAction)
  
  rospy.loginfo('Waiting for needed services')
  rospy.wait_for_service('/bb_estimator/estimate_bb_alt')
  rospy.wait_for_service('/interaction_primitives/set_pregrasp_position')
  
  rospy.loginfo("Waiting for server...")
  client.wait_for_server()
  goal = ManualBBEstimationGoal()
  #goal.grasp_type = "open"
  #goal.max_force = 100
  client.send_goal(goal,feedback_cb=bb_est_feedback)
  
  rospy.loginfo("Waiting for result...")
  client.wait_for_result()
  
  result = client.get_result()
  
  print result
  
  rospy.loginfo('Action finished')
  
  remove_im()
  
  #print result.grasped
  #print result.tip1_force
  #print result.tip2_force
  #print result.tip3_force
  
  #rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())

  
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
