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

import roslib; roslib.load_manifest('srs_arm_navigation_tests')
import rospy
import smach
import smach_ros
import actionlib

from math import fabs, sqrt
import copy
from numpy import power

from geometry_msgs.msg import Vector3, PoseStamped, Pose
from std_msgs.msg import ColorRGBA

from srs_assisted_arm_navigation_msgs.msg import *
from srs_assisted_arm_navigation_msgs.srv import ArmNavCollObj, ArmNavMovePalmLink
from srs_assisted_grasping_msgs.srv import GraspingAllow

from srs_interaction_primitives.srv import AddObject, RemovePrimitive, SetPreGraspPosition, RemovePreGraspPosition, GetUnknownObject, SetAllowObjectInteraction, AddUnknownObject
from srs_interaction_primitives.msg import MoveArmToPreGrasp, PoseType 
from srs_env_model_percp.srv import EstimateBBAlt
from srs_object_database_msgs.srv import GetMesh, GetObjectId

from shared_state_information import *
from arm_manip_helper_methods import *
    

class grasp_unknown_object_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'],
                         input_keys=[''],
                         output_keys=[''])
    
    global listener
    
    self.hlp = common_helper_methods()
    
    self.object_added = False
    
    self.object_pose = None
    self.object_bb = None
    
    but_gui_ns = '/interaction_primitives'
    
    arm_manip_ns = '/but_arm_manip'
    self.arm_action_name = arm_manip_ns + '/manual_arm_manip_action'
    self.s_grasping_allow = arm_manip_ns + '/grasping_allow'
    self.s_allow_interaction = but_gui_ns + '/set_allow_object_interaction'
    
    self.s_bb_est = '/bb_estimator/estimate_bb_alt'
    self.s_add_unknown_object = but_gui_ns + '/add_unknown_object'
    self.s_get_object = but_gui_ns + '/get_unknown_object'
    
    self.s_coll_obj = arm_manip_ns + '/arm_nav_coll_obj';
    
    self.unknown_object_name='unknown_object'
    self.unknown_object_description='Unknown object to grasp'  
  
  def bb_est_feedback(self,feedback):
    
    rospy.loginfo('Feedback received')
    
    est = rospy.ServiceProxy('/bb_estimator/estimate_bb_alt',EstimateBBAlt)
      
    header=rospy.Header() 
   
    header.frame_id = '/map'
    header.stamp = feedback.timestamp
    
    rospy.loginfo('Calling bb est srv')
      
    try:
          
      est_result = est(header=header,
                       p1=feedback.p1,
                       p2=feedback.p2,
                       mode=2)
          
    except Exception, e:
          
      rospy.logerr("Error on calling service: %s",str(e))
      return
  
    rospy.loginfo('BB estimation performed!')
 

    if self.object_added == True:
        
        if self.hlp.remove_im(self.unknown_object_name) == True:
            
            self.object_added = False
            
 
    if self.object_added == False:
 
        add_object = rospy.ServiceProxy(self.s_add_unknown_object, AddUnknownObject)
        rospy.loginfo('Calling %s service',self.s_add_unknown_object)
           
        try:
          
            #===================================================================
            # add_object(frame_id = '/map',
            #           name = self.unknown_object_name,
            #           description = self.unknown_object_description,
            #           pose = est_result.pose,
            #           bounding_box_lwh = est_result.bounding_box_lwh,
            #           color = color,
            #           use_material = False)
            #===================================================================
            
            add_object(frame_id='/map',
                       name=self.unknown_object_name,
                       description=self.unknown_object_description,
                       pose_type= PoseType.POSE_BASE,
                       pose = est_result.pose,
                       scale = est_result.bounding_box_lwh)
            
            self.object_added = True
            
            rospy.loginfo('IM added')
          
          
        except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
          
        # allow interaction for this object  
        self.hlp.set_interaction(self.unknown_object_name,True)
      
    else:
        
        rospy.logerr('Could not add new IM - old one was not removed!');
        
    return
  
  
  def get_im_pose(self):
      
      get_object = rospy.ServiceProxy(self.s_get_object, GetUnknownObject)
      
      try:
             
             res = get_object(name=self.unknown_object_name)
             
             if res.frame_id is not ('map' or '/map'):
                 
                rospy.logwarn('TODO: Transformation of IM pose needed! Frame_id: %s',res.frame_id)
             
             self.object_pose = res.pose
             self.object_bb = res.scale        
          
      except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e)) 
          
      if self.object_pose is not None:
          
          # conversion to lwh
          #self.object_pose.position.y -= res.scale.y/2.0
          self.object_pose.position.z -= res.scale.z/2.0
          
          
          return True
      else:
          
          return False
      
      
  
  def add_im_for_object(self):
   
   # /but_arm_manip/manual_bb_estimation_action
   roi_client = actionlib.SimpleActionClient('/but_arm_manip/manual_bb_estimation_action',ManualBBEstimationAction)
   
   rospy.loginfo("Waiting for ROI action server...")
   roi_client.wait_for_server()
  
   goal = ManualBBEstimationGoal()
   goal.object_name = self.unknown_object_name
   
   roi_client.send_goal(goal,feedback_cb=self.bb_est_feedback)
  
   rospy.loginfo("Waiting for result...")
   roi_client.wait_for_result()
      
   result = roi_client.get_result()
      
   print result
   
   # not let's suppose that IM is already on its position.... 
      
   rospy.loginfo('Action finished')
   
  def add_bb_to_planning(self):
      
    coll_obj = rospy.ServiceProxy(self.s_coll_obj, ArmNavCollObj);
    
    mpose = PoseStamped()
    
    mpose.header.frame_id = '/map'
    mpose.pose = self.object_pose
    
    try:
      
      coll_obj(object_name = self.unknown_object_name,
               pose = mpose,
               bb_lwh = self.object_bb,
               allow_collision=True,
               attached=False,
               attach_to_frame_id='');
      
    except Exception, e:
      
      rospy.logerr('Cannot add unknown object to the planning scene, error: %s',str(e))
      
  def change_bb_to_attached(self):
      
      rospy.loginfo('Setting object bb to be attached (not implemented)')
      
    
  def execute(self,userdata):
      
    global listener
    
    rospy.loginfo('Waiting for needed services and topics...')
    
    if self.hlp.wait_for_srv(self.s_grasping_allow) is False:
         return 'failed'
     
    if self.hlp.wait_for_srv(self.s_allow_interaction) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_bb_est) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_add_unknown_object) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_get_object) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_coll_obj) is False:
        return 'failed'
    
    
    self.s_coll_obj
    
    grasping_client = actionlib.SimpleActionClient('/but_arm_manip/manual_arm_manip_action',ManualArmManipAction)
  
    rospy.loginfo("Waiting for grasping action server...")
    grasping_client.wait_for_server()
     
    # ask user to select and tune IM for unknown object
    self.add_im_for_object()
    
    if not self.get_im_pose():
        
        rospy.logerr('Could not get position of unknown object. Lets try to continue without it...')
        
    else:
        
        self.add_bb_to_planning()
        
        
    
    # send grasping goal and allow grasping buttons :)
    goal = ManualArmManipGoal()
      
    goal.allow_repeat = False
    goal.action = "Grasp object and put it on tray"
    goal.object_name = self.unknown_object_name
      
    grasping_client.send_goal(goal)
      
    # call allow grasping service
    rospy.wait_for_service('/but_arm_manip/grasping_allow')
      
          
    grasping_allow = rospy.ServiceProxy('/but_arm_manip/grasping_allow',GraspingAllow)
      
    try:
          
      gr = grasping_allow(allow=True)
          
    except Exception, e:
          
      rospy.logerr("Error on calling service: %s",str(e))
      return 'failed'
  
 
    rospy.loginfo("Waiting for result")
    grasping_client.wait_for_result()
  
    result = grasping_client.get_result()
    
    self.hlp.remove_im(self.unknown_object_name)    
    
    if result.success:
        
        rospy.loginfo('Object should be on tray.')
        return 'completed'
    
    if result.failed:
        
        rospy.logwarn('Operator was not able to grasp object and put it on tray')
        return 'failed'
    
    if result.timeout:
        
        rospy.logwarn('Action was too long')
        return 'not_completed'
    
    
    rospy.logerr('This should never happen!')
    return 'failed'  
    
