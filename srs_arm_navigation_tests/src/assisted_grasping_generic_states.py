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
from srs_assisted_arm_navigation_msgs.srv import ArmNavCollObj, ArmNavMovePalmLink, ArmNavRemoveCollObjects, ArmNavSetAttached
from srs_assisted_grasping_msgs.srv import GraspingAllow

from srs_interaction_primitives.srv import AddObject, RemovePrimitive, SetPreGraspPosition, RemovePreGraspPosition, GetUnknownObject, SetAllowObjectInteraction, AddUnknownObject
from srs_interaction_primitives.msg import MoveArmToPreGrasp, PoseType 
from srs_env_model_percp.srv import EstimateBBAlt
from srs_object_database_msgs.srv import GetMesh, GetObjectId
from srs_env_model.srv import LockCollisionMap

from shared_state_information import *
from arm_manip_helper_methods import *
from simple_script_server import *
#from cob_script_server.msg import *

from cob_srvs.srv import SetJointStiffness

sss = simple_script_server()


class detect_unknown_object_assisted(smach.State):
 def __init__(self):
     
   smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'],
                        input_keys=[''],
                        output_keys=['object_name','object_description','object_bb_pose','object_bb'])
   
   global listener
    
   self.hlp = common_helper_methods()
    
   self.object_added = False
    
   self.object_pose = None
   self.object_bb = None
    
   but_gui_ns = '/interaction_primitives'
   
   arm_manip_ns = '/but_arm_manip'
   self.s_allow_interaction = but_gui_ns + '/set_allow_object_interaction'
   
   self.s_bb_est = '/bb_estimator/estimate_bb_alt'
   self.s_add_unknown_object = but_gui_ns + '/add_unknown_object'
   self.s_get_object = but_gui_ns + '/get_unknown_object'
    
   self.s_coll_obj = arm_manip_ns + '/arm_nav_coll_obj';
    
   self.unknown_object_name='unknown_object'
   self.unknown_object_description='Unknown object to grasp'  
   
   self.disable_bb_video = rospy.get_param('~bb/disable_video',False) # TDDO set this to False after testing and preparation of launch file
   
   self.bb_pos_x = rospy.get_param('~bb/pos_x',-0.5)
   self.bb_pos_y = rospy.get_param('~bb/pos_y',0.0)
   self.bb_pos_z = rospy.get_param('~bb/pos_z',1.3)
   
   self.bb_lwh_x = rospy.get_param('~bb/lwh_x',0.1)
   self.bb_lwh_y = rospy.get_param('~bb/lwh_y',0.1)
   self.bb_lwh_z = rospy.get_param('~bb/lwh_z',0.1)
   
 def execute(self,userdata):
       
   rospy.loginfo('Assisted detection of unknown object')
   
   if self.hlp.wait_for_srv(self.s_allow_interaction) is False:
    return 'failed'
    
   if self.hlp.wait_for_srv(self.s_bb_est) is False:
       return 'failed'
    
   if self.hlp.wait_for_srv(self.s_add_unknown_object) is False:
       return 'failed'
    
   if self.hlp.wait_for_srv(self.s_get_object) is False:
       return 'failed'
    
   self.object_added = False
   self.object_pose = None
   self.object_bb = None
    
   # ask user to select and tune IM for unknown object
   self.add_im_for_object()
    
   if not self.get_im_pose():
        
       rospy.logerr('Could not get position of unknown object.')
       self.hlp.remove_im(self.unknown_object_name)  
       userdata.object_name = self.unknown_object_name
       userdata.object_description = self.unknown_object_description
       userdata.object_bb_pose = None
       userdata.object_bb = None
       self.object_added = False
       return 'not_completed'
        
   self.hlp.remove_im(self.unknown_object_name)  
   
   userdata.object_name = self.unknown_object_name
   userdata.object_description = self.unknown_object_description
   userdata.object_bb_pose = self.object_pose
   userdata.object_bb = self.object_bb
   
   rospy.loginfo('We have manually detected object.')
   
   return 'completed'
    
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
            
        else:
            
            rospy.logerr("Error on removing IM. Grrr.")
            self.object_added = False
            
 
    if self.object_added == False:
 
        add_object = rospy.ServiceProxy(self.s_add_unknown_object, AddUnknownObject)
        rospy.loginfo('Calling %s service',self.s_add_unknown_object)

        print "POSE of unknown object from BB estimator"
        print est_result.pose
           
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
                       scale = est_result.bounding_box_lwh,
                       disable_material=True)
            
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
             
             #if res.frame_id is not ('map' or '/map'):
                 
             #   rospy.logwarn('TODO: Transformation of IM pose needed! Frame_id: %s',res.frame_id)
             
             self.object_pose = res.pose
             self.object_bb = res.scale        
          
      except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e)) 

          self.object_pose = None
          self.object_bb = None
          
      if self.object_pose is not None:
          
          # conversion to lwh
          #self.object_pose.position.y -= res.scale.y/2.0
          self.object_pose.position.z -= res.scale.z/2.0
          
          
          return True
      else:
          
          return False
      
      
  
 def add_im_for_object(self):
     
   global listener
   
   # /but_arm_manip/manual_bb_estimation_action
   roi_client = actionlib.SimpleActionClient('/but_arm_manip/manual_bb_estimation_action',ManualBBEstimationAction)
   
   rospy.loginfo("Waiting for ROI action server...")
   roi_client.wait_for_server()
  
   goal = ManualBBEstimationGoal()
   goal.object_name = self.unknown_object_name
   goal.disable_video = self.disable_bb_video
   
   roi_client.send_goal(goal,feedback_cb=self.bb_est_feedback)
   
   if self.disable_bb_video:
       
       # normally, object is inserted in feedback, now we have to do it manually
       add_object = rospy.ServiceProxy(self.s_add_unknown_object, AddUnknownObject)
       rospy.loginfo('Calling %s service',self.s_add_unknown_object)
    
       bb_pose = PoseStamped()
       
       bb_pose.header.frame_id = '/base_link'
       bb_pose.header.stamp = rospy.Time(0)
       
       bb_pose.pose.position.x = self.bb_pos_x
       bb_pose.pose.position.y = self.bb_pos_y
       bb_pose.pose.position.z = self.bb_pos_z
       
       bb_pose.pose.orientation.x = 0.0
       bb_pose.pose.orientation.y = 0.0
       bb_pose.pose.orientation.z = 0.0
       bb_pose.pose.orientation.w = 1.0
       
       listener.waitForTransform('/map',bb_pose.header.frame_id,bb_pose.header.stamp,rospy.Duration(10))
       
       bb_pose = listener.transformPose('/map',bb_pose)
       
       bb_lwh = Vector3()
       
       bb_lwh.x = self.bb_lwh_x
       bb_lwh.y = self.bb_lwh_y
       bb_lwh.z = self.bb_lwh_z
    
       try:
            
            add_object(frame_id='/map',
                       name=self.unknown_object_name,
                       description=self.unknown_object_description,
                       pose_type= PoseType.POSE_BASE,
                       pose = bb_pose.pose,
                       scale = bb_lwh,
                       disable_material=True)
            
            self.object_added = True
            
            rospy.loginfo('IM added')
          
          
       except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
          
          
       self.hlp.set_interaction(self.unknown_object_name,True)
  
   rospy.loginfo("Waiting for result...")
   roi_client.wait_for_result()
      
   result = roi_client.get_result()
      
   print result
   
   # not let's suppose that IM is already on its position.... 
      
   rospy.loginfo('Action finished')   


    

class grasp_unknown_object_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','repeat_detection','pre-empted'],
                         input_keys=['object_name','object_description','object_bb_pose','object_bb'],
                         output_keys=[''])
    
    global listener
    
    self.hlp = common_helper_methods()
    
    but_gui_ns = '/interaction_primitives'
    
    arm_manip_ns = '/but_arm_manip'
    self.arm_action_name = arm_manip_ns + '/manual_arm_manip_action'
    self.s_grasping_allow = arm_manip_ns + '/grasping_allow'
    
    self.s_coll_obj = arm_manip_ns + '/arm_nav_coll_obj'
    self.s_rem_coll_obj = arm_manip_ns + '/arm_rem_coll_obj'
    self.s_set_attached = arm_manip_ns + '/arm_nav_set_attached'
    
    self.s_set_stiffness = "/arm_controller/set_joint_stiffness"
    
    self.s_lock_coll_map = '/but_env_model/lock_collision_map'
   
  def add_bb_to_planning(self):
      
    coll_obj = rospy.ServiceProxy(self.s_coll_obj, ArmNavCollObj);
    
    mpose = PoseStamped()
    
    mpose.header.frame_id = '/map'
    mpose.pose = self.object_pose

    print "POSE of object which Im going to add to planning"
    print mpose
    
    try:
      
      coll_obj(object_name = self.unknown_object_name,
               pose = mpose,
               bb_lwh = self.object_bb,
               allow_collision=True,
               attached=False,
               attach_to_frame_id='',
               allow_pregrasps=False);
      
    except Exception, e:
      
      rospy.logerr('Cannot add unknown object to the planning scene, error: %s',str(e))
      
  def change_bb_state(self,is_attached):
      
      if is_attached:
          
          rospy.loginfo('Setting object bb to be attached')
          
      else:
         
         rospy.loginfo('Setting object bb to be NOT attached')
      
      att = rospy.ServiceProxy(self.s_set_attached,ArmNavSetAttached)
      
      try:
          
        res = att(object_name=self.userdata.object_name,
                  attached = is_attached)
          
      except Exception, e:
          
        rospy.logerr("Error on calling service: %s",str(e))
      
      
  def set_grasping_state(self,enabled):
      
      grasping_allow = rospy.ServiceProxy('/but_arm_manip/grasping_allow',GraspingAllow)
      
      try:
          
          gr = grasping_allow(allow=enabled)
          
      except Exception, e:
          
          rospy.logerr("Error on calling service: %s",str(e))
    
  def execute(self,userdata):
      
    global listener
    
    self.object_pose = userdata.object_bb_pose
    self.object_bb = userdata.object_bb
    self.unknown_object_name = userdata.object_name
    self.unknown_object_description = userdata.object_description
    
    rospy.loginfo('Waiting for needed services and topics...')
    
    if self.hlp.wait_for_srv(self.s_grasping_allow) is False:
         return 'failed'
    
    if self.hlp.wait_for_srv(self.s_coll_obj) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_rem_coll_obj) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_set_attached) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_set_stiffness) is False:
        return 'failed'
    
    if self.hlp.wait_for_srv(self.s_lock_coll_map) is False:
        return 'failed'
    
    self.userdata = userdata
    
    lock_coll_map = rospy.ServiceProxy(self.s_lock_coll_map,LockCollisionMap)
    
    rospy.loginfo('Locking collision map.')
    try:
          
        res = lock_coll_map(lock = True)
          
    except Exception, e:
          
        rospy.logerr("Error on calling service: %s",str(e))
    
    # open gripper to cylopen
    #script_server = actionlib.SimpleActionClient('',ScriptAction)
    rospy.loginfo('Opening gripper')
    sss.move('sdh', 'cylopen', True)
    
    # first, try to add BB to planning
    self.add_bb_to_planning()
    
    # call allow grasping service
    #rospy.wait_for_service('/but_arm_manip/grasping_allow')
    
    arm_stiff = rospy.ServiceProxy(self.s_set_stiffness,SetJointStiffness)
    
    rospy.loginfo("Setting arm stiffness")
    try:
          
        st = arm_stiff([300,300,300,300,200,200,200])
          
    except Exception, e:
          
        rospy.logerr("Error on calling service: %s",str(e))  
    
    arm_nav_client = actionlib.SimpleActionClient('/but_arm_manip/manual_arm_manip_action',ManualArmManipAction)
  
    rospy.loginfo("Waiting for grasping action server...")
    arm_nav_client.wait_for_server()
    
    # send grasping goal and allow grasping buttons :)
    grasp_goal = ManualArmManipGoal()
      
    grasp_goal.allow_repeat = True
    grasp_goal.action = "Move arm to suitable position and then try to grasp object."
    grasp_goal.object_name = self.unknown_object_name
      
    arm_nav_client.send_goal(grasp_goal)
    
    self.set_grasping_state(True)
    
    rospy.loginfo("Waiting for result")
    arm_nav_client.wait_for_result()
      
 
    result = arm_nav_client.get_result()
    
    if result.failed:
    
     rospy.logwarn('Operator was not able to grasp object.')
     self.set_grasping_state(False)
     return 'failed' 

    if result.repeat:
        
      rospy.loginfo('Operator wants new detection of object.')
      self.set_grasping_state(False)
      return 'repeat_detection'
    
    
    if not result.success:
        
      rospy.logerr('Weird state...')
      self.set_grasping_state(False)
      return 'failed'
    
    # object should be grasped, so we can attach it to gripper
    self.change_bb_state(is_attached=True)
    
    m_goal = ManualArmManipGoal()
    m_goal.allow_repeat = False
    m_goal.action = "Navigate arm and place object on tray. Object will be attached to the arm. When finished, gripper will be opened."
    m_goal.object_name = self.unknown_object_name
    
    arm_nav_client.send_goal(m_goal)
    arm_nav_client.wait_for_result()
    
    m_result = arm_nav_client.get_result()
    
    self.set_grasping_state(False)
      
    # object should be grasped, so we can attach it to gripper
    self.change_bb_state(is_attached=False)
      
    if m_result.failed:
        
        rospy.loginfo('Operator failed to place object on the tray.')
        return 'failed'
    
    if not m_result.success:
        
        rospy.loginfo('Weird state. Again...')
        return 'failed'
    

    # object is on tray and we should navigate arm to some "home" position
    # but first, we will open the gripper
    rospy.loginfo('Opening gripper')
    sss.move('sdh', 'cylopen', True)

    a_goal = ManualArmManipGoal()
    a_goal.allow_repeat = False
    a_goal.action = "Navigate arm from object."
    a_goal.object_name = self.unknown_object_name
    
    arm_nav_client.send_goal(a_goal)
    arm_nav_client.wait_for_result()
    
    a_result = arm_nav_client.get_result()
    
    if a_result.failed:
        
        rospy.loginfo('Operator was not able to navigate arm from object...')
        return 'failed'
    
    if not a_result.success:
        
        rospy.logerr('Again... This should not happen.')
        return 'failed'


    remove_coll_objects = rospy.ServiceProxy(self.s_rem_coll_obj,ArmNavRemoveCollObjects)
    
    try:
          
      rr = remove_coll_objects()
          
    except Exception, e:
          
      rospy.logerr("Error on calling service: %s",str(e))
      
      
    
    
    rospy.loginfo('Unlocking collision map.')
    try:
          
        res = lock_coll_map(lock = False)
          
    except Exception, e:
          
        rospy.logerr("Error on calling service: %s",str(e))
      
      
    # arm is away and object is cleared... we can end
    rospy.loginfo('State is finishing.')
    return 'completed'
    
