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
from srs_assisted_arm_navigation.msg import *
from srs_interaction_primitives.srv import AddObject
from srs_interaction_primitives.srv import RemovePrimitive
from srs_interaction_primitives.srv import SetPreGraspPosition
from srs_interaction_primitives.srv import RemovePreGraspPosition
from srs_interaction_primitives.msg import MoveArmToPreGrasp
from math import fabs
from math import sqrt
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from srs_object_database_msgs.srv import GetMesh
from srs_object_database_msgs.srv import GetObjectId
#from srs_object_database_msgs.srv import GetMesh
#from srs_object_database_msgs.srv import GetObjectId
from arm_navigation_msgs.msg import CollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose
#from tf import TransformListener
#import threading
from srs_assisted_arm_navigation.srv import ArmNavCollObj
from srs_assisted_arm_navigation.srv import ArmNavMovePalmLink
import copy
from shared_state_information import *
from numpy import power


class grasp_unknown_object_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'],
                         input_keys=[''],
                         output_keys=[''])
    
    global listener
 
    but_gui_ns = '/interaction_primitives'
    
    arm_manip_ns = '/but_arm_manip'
    self.arm_action_name = arm_manip_ns + '/manual_arm_manip_action'
    self.s_grasping_allow = arm_manip_ns + '/grasping_allow'
    
  
  def add_im_for_object(self):
   
   # /but_arm_manip/manual_bb_estimation_action
   roi_client = actionlib.SimpleActionClient('/but_arm_manip/manual_bb_estimation_action',ManualBBEstimationAction)
   
      
    
  def execute(self,userdata):
      
    global listener
    
    grasping_client = actionlib.SimpleActionClient('/but_arm_manip/manual_arm_manip_action',ManualArmManipAction)
  
    rospy.loginfo("Waiting for server...")
    grasping_client.wait_for_server()
    goal = ManualArmManipGoal()
      
    goal.allow_repeat = False
    goal.action = "Grasp object and put it on tray"
    goal.object_name = "Unknown object"
      
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
    
    

class move_arm_to_given_positions_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted','repeat'],
                         input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object','pose_of_the_target_object','bb_of_the_target_object'],
                         output_keys=['id_of_the_reached_position'])
    
    global listener
    #listener = TransformListener()
 
    but_gui_ns = '/interaction_primitives'
    self.s_set_gr_pos = but_gui_ns + '/set_pregrasp_position'
    self.s_add_object = but_gui_ns + '/add_object'
    self.s_remove_object = but_gui_ns + '/remove_primitive'
    
    # object database
    self.s_get_object_id = '/get_models'
    self.s_get_model_mesh = '/get_model_mesh'
    
    #ns = rospy.get_param('/but_arm_manip','/arm_manip_namespace')
    arm_manip_ns = '/but_arm_manip'
    #action_name = rospy.get_param('/manual_arm_manip_action','~arm_action_name')
    self.action_name = arm_manip_ns + '/manual_arm_manip_action'
    self.s_coll_obj = arm_manip_ns + '/arm_nav_coll_obj';
    self.s_move_arm = arm_manip_ns + '/arm_nav_move_palm_link';
    
      
  def add_grpos(self,userdata):
      
    global listener
  
    rospy.loginfo("Waiting for %s service",self.s_set_gr_pos)
    rospy.wait_for_service(self.s_set_gr_pos)
    set_gr_pos = rospy.ServiceProxy(self.s_set_gr_pos, SetPreGraspPosition)
    rospy.loginfo('Calling service %s',self.s_set_gr_pos)
    
    pregr_tmp = list(userdata.list_of_target_positions)
    
    for idx in range(0,len(userdata.list_of_id_for_target_positions)):
    
      try:
        
        gpose = Pose()
        
        gpose = pregr_tmp[idx].pre_grasp.pose
        
        # shift position to be relative to detected object 
        gpose.position.x -= userdata.pose_of_the_target_object.pose.position.x
        gpose.position.y -= userdata.pose_of_the_target_object.pose.position.y
        gpose.position.z -= userdata.pose_of_the_target_object.pose.position.z
        
        
        rospy.loginfo('Adding gr pos id %d [x=%f,y=%f,z=%f]',userdata.list_of_id_for_target_positions[idx],gpose.position.x,gpose.position.y,gpose.position.z)
        
        res = set_gr_pos(name=userdata.name_of_the_target_object,
                         pos_id=idx+1,
                         pose=gpose)
        
      except Exception, e:
        
        rospy.logerr('Cannot add gr pos IM for pos ID: %d, error: %s',idx+1,str(e))
    
    
    
  def add_im(self,userdata):
      
    global listener
    
    #listener = TransformListener()
    
    rospy.loginfo("Waiting for %s service",self.s_get_object_id)
    rospy.wait_for_service(self.s_get_object_id)
    get_object_id = rospy.ServiceProxy(self.s_get_object_id, GetObjectId)
    rospy.loginfo('Calling service %s',self.s_get_object_id)
  
    obj_db_id = None
  
    try:
  
      res = get_object_id(type=userdata.name_of_the_target_object)
      
      #obj_db_id = int(res.model_ids[0])
      idx = res.model_category.index(userdata.name_of_the_target_object)
      obj_db_id = int(res.model_ids[idx])
      
      rospy.loginfo('Object name (%s) successfully converted to ID (%d)',userdata.name_of_the_target_object,obj_db_id)
    
    except Exception, e:
    
      rospy.logerr('Error on 0 name (%s) to ID... Lets use ID=1. Error: %s',userdata.name_of_the_target_object,str(e))
      obj_db_id = 1
    
    shape = None
    mesh = 'package://cob_gazebo_objects/Media/models/milk.dae'
    #db_shape = None
    use_default_mesh = True
    
    rospy.loginfo("Waiting for %s service",self.s_get_model_mesh)
    rospy.wait_for_service(self.s_get_model_mesh)
    get_model_mesh = rospy.ServiceProxy(self.s_get_model_mesh, GetMesh)
    rospy.loginfo('Calling service %s (with ID=%d)',self.s_get_model_mesh,obj_db_id)
    
    
    try:
      
      object_shape = get_model_mesh(model_ids=[obj_db_id])
      shape = object_shape.msg[0].mesh
      use_default_mesh = False
      
    except Exception, e:
      
      rospy.logerr('Cannot get mesh from db. We will use default one for milkbox. Error: %s',str(e))
      
       
    rospy.loginfo("Waiting for %s service",self.s_add_object)
    rospy.wait_for_service(self.s_add_object)
    add_object = rospy.ServiceProxy(self.s_add_object, AddObject)
    rospy.loginfo('Calling %s service',self.s_add_object)

    # color of the bounding box
    color = ColorRGBA()
    color.r = 1
    color.g = 1
    color.b = 0
    color.a = 1
      
    bpose = copy.deepcopy(self.pose_of_the_target_object_in_map.pose)
    bpose.position.z += userdata.bb_of_the_target_object['bb_lwh'].z/2
    
    # TODO remove next line - it was just for testing!!!!!!!!
    use_default_mesh = True
    
    try:
      
      if use_default_mesh:
      
        add_object(frame_id = '/map',
                   name = userdata.name_of_the_target_object,
                   description = 'Object to grasp',
                   pose = bpose,
                   bounding_box_lwh = userdata.bb_of_the_target_object['bb_lwh'],
                   color = color,
                   resource = 'package://cob_gazebo_objects/Media/models/milk.dae',
                   #shape = shape,
                   use_material = True)
      else:
        
        add_object(frame_id = '/map',
                   name = userdata.name_of_the_target_object,
                   description = 'Object to grasp',
                   pose = bpose,
                   bounding_box_lwh = userdata.bb_of_the_target_object['bb_lwh'],
                   color = color,
                   #resource = 'package://cob_gazebo_objects/Media/models/milk.dae', 
                   shape = shape,
                   use_material = True)
      
      
    except Exception, e:
      
      rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
      
    return shape
    
    
  def remove_im(self,userdata):
      
    global listener
    
    # clean-up : removing interactive marker...
    rospy.loginfo("Waiting for %s service",self.s_remove_object)
    rospy.wait_for_service(self.s_remove_object)
    remove_object = rospy.ServiceProxy(self.s_remove_object, RemovePrimitive)
    
    try:
    
      remove_object(name=userdata.name_of_the_target_object)
      
    except Exception, e:
      
      rospy.logerr('Cannot remove IM object from the scene, error: %s',str(e))
 
 
  def pregr_im_callback(self,data):
      
    global listener
    
    rospy.loginfo("Move arm to pregrasp pos. ID=%d, object=%s, x=%f, y=%f, z=%f",
                  data.pos_id,
                  data.marker_name,
                  self.userdata.list_of_target_positions[data.pos_id-1].pre_grasp.pose.position.x,
                  self.userdata.list_of_target_positions[data.pos_id-1].pre_grasp.pose.position.y,
                  self.userdata.list_of_target_positions[data.pos_id-1].pre_grasp.pose.position.z);
    
    rospy.wait_for_service(self.s_move_arm)
    move_arm = rospy.ServiceProxy(self.s_move_arm, ArmNavMovePalmLink);
    
    print "test - list_of_trg_pos"
    print self.userdata.list_of_target_positions[data.pos_id-1]
    
    target_pos = []
    #target_pos = list(self.userdata.list_of_target_positions)
    target_pos = copy.deepcopy(self.userdata.list_of_target_positions)
    
    pose_of_the_target_object_in_base_link = None
    # self.pose_of_the_target_object_in_map -> transform it to /base_link
    
    t = rospy.Time(0)

    transf_target = '/base_link'

    listener.waitForTransform(transf_target,'/map',t,rospy.Duration(5))
  
    if listener.canTransform(transf_target,'/map',t):
    
      pose_of_the_target_object_in_base_link = listener.transformPose(transf_target,self.pose_of_the_target_object_in_map)
    
    else:
    
      rospy.logerr('Transformation is not possible!')
      sys.exit(0) 
    
    target_pos[data.pos_id-1].pre_grasp.pose.position.x += pose_of_the_target_object_in_base_link.pose.position.x
    target_pos[data.pos_id-1].pre_grasp.pose.position.y += pose_of_the_target_object_in_base_link.pose.position.y
    target_pos[data.pos_id-1].pre_grasp.pose.position.z += (pose_of_the_target_object_in_base_link.pose.position.z + self.userdata.bb_of_the_target_object['bb_lwh'].z/2)
    
  
    print "Moving arm's IM to this position:"
    print target_pos[data.pos_id-1]
    
    try:
      
      move_arm(sdh_palm_link_pose=target_pos[data.pos_id-1].pre_grasp.pose);
      
    except Exception, e:
      
      rospy.logerr('Cannot move sdh_palm_link to given position, error: %s',str(e))
      
    del target_pos
    
    
  
  def execute(self,userdata):
      
    global listener
           
           
    if len(userdata.list_of_target_positions)==0:
      
      rospy.logerr('There are NO grasping target positions')
      return 'failed'
    
    if len(userdata.list_of_target_positions) != len(userdata.list_of_id_for_target_positions):
      
      rospy.logerr('List with gr. positions has different length than list with IDs')
      return 'failed'
    
    # try to transform pose of detected object from /base_link to /map
    transf_target = '/map'
  
    rospy.loginfo('Lets transform pose of object from %s to %s frame',userdata.pose_of_the_target_object.header.frame_id,transf_target)

    t = rospy.Time(0)
    
    userdata.id_of_the_reached_position = -1
    
  
    pose_of_target = copy.deepcopy(userdata.pose_of_the_target_object)
    
  
    rospy.loginfo('Waiting for transform for some time...')
    listener.waitForTransform(transf_target,pose_of_target.header.frame_id,t,rospy.Duration(10))
  
    if listener.canTransform(transf_target,pose_of_target.header.frame_id,t):
    
      self.pose_of_the_target_object_in_map = listener.transformPose(transf_target,pose_of_target)
    
    else:
    
      rospy.logerr('Transformation is not possible!')
      return 'failed'
      
    
    self.userdata = userdata
        
    rospy.loginfo('Executing state move_arm_to_given_positions_assisted (%s)',userdata.name_of_the_target_object)    
       
    db_shape = None
    
    # add IM to the scene
    db_shape = self.add_im(userdata)
    
    # add IM for grasping positions
    self.add_grpos(userdata)
    
    pregr_topic = "/interaction_primitives/" + userdata.name_of_the_target_object + "/update/move_arm_to_pregrasp"
    rospy.loginfo("Subscribing to %s topic",pregr_topic);
    rospy.Subscriber(pregr_topic, MoveArmToPreGrasp, self.pregr_im_callback)
    
    rospy.loginfo("Waiting for %s service",self.s_coll_obj)
    rospy.wait_for_service(self.s_coll_obj)
    coll_obj = rospy.ServiceProxy(self.s_coll_obj, ArmNavCollObj);
    
    try:
      
      coll_obj(object_name = userdata.name_of_the_target_object,
               pose = userdata.pose_of_the_target_object,
               bb_lwh = userdata.bb_of_the_target_object['bb_lwh']);
      
    except Exception, e:
      
      rospy.logerr('Cannot add detected object to the planning scene, error: %s',str(e))
    

    # lets start action...    
    client = actionlib.SimpleActionClient(self.action_name,ManualArmManipAction)
    
    rospy.loginfo("Waiting for actionlib server...")
    client.wait_for_server()
    
    goal = ManualArmManipGoal()
       
    goal.allow_repeat = True
    goal.action = "Move arm to suitable pregrasp position"    
    goal.object_name = userdata.name_of_the_target_object
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    
    rospy.loginfo("Waiting for result")
    client.wait_for_result()
    rospy.loginfo("I have result!! :-)")
    
    result = client.get_result()
    
    # clean up
    self.remove_im(userdata)
    
    rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())
    
    if result.success:
      rospy.loginfo('Hooray, successful action. Lets find ID of reached position.')
      # TODO find closest pregrasp position and send back its ID! 
      
      sdh_ps = PoseStamped()
      
      sdh_ps.header.frame_id = '/sdh_palm_link'
      sdh_ps.header.stamp = rospy.Time.now()
      sdh_ps.pose.position.x = 0
      sdh_ps.pose.position.y = 0
      sdh_ps.pose.position.z = 0
      
      listener.waitForTransform('/map',sdh_ps.header.frame_id,sdh_ps.header.stamp,rospy.Duration(10))
  
      if listener.canTransform('/map',sdh_ps.header.frame_id,sdh_ps.header.stamp):
    
        sdh_ps_tr = listener.transformPose('/map',sdh_ps)
    
      else:
    
        rospy.logerr('Transformation is not possible!')
        userdata.id_of_the_reached_position = -1
        return 'not_completed'
      
      print "sdh_palm_link pose in /map coord. system"
      print sdh_ps_tr
      
      print "object pose in /map coord"
      print self.pose_of_the_target_object_in_map
      
      min_dist = 1000
      min_id = -1
      
      pregrasp_pose_in_map = Vector3()
      closest_one = Vector3()
      
      for idx in range(len(userdata.list_of_target_positions)):
        
        pregrasp_pose_in_map.x = self.pose_of_the_target_object_in_map.pose.position.x + userdata.list_of_target_positions[idx].pre_grasp.pose.position.x
        pregrasp_pose_in_map.y = self.pose_of_the_target_object_in_map.pose.position.y + userdata.list_of_target_positions[idx].pre_grasp.pose.position.y
        pregrasp_pose_in_map.z = self.pose_of_the_target_object_in_map.pose.position.z + userdata.list_of_target_positions[idx].pre_grasp.pose.position.z + self.userdata.bb_of_the_target_object['bb_lwh'].z/2
        
        print "PREGRASP position in /map"
        print idx
        print pregrasp_pose_in_map
      
        dist = sqrt( power((sdh_ps_tr.pose.position.x - pregrasp_pose_in_map.x),2) +
                power((sdh_ps_tr.pose.position.y - pregrasp_pose_in_map.y),2) + 
                power((sdh_ps_tr.pose.position.z - pregrasp_pose_in_map.z),2) );
                
        rospy.loginfo('Position ID=%d, distance=%fm',userdata.list_of_id_for_target_positions[idx],dist)
                
        if dist < min_dist:
          
          min_dist = dist;
          min_id = userdata.list_of_id_for_target_positions[idx]
          closest_one = copy.deepcopy(pregrasp_pose_in_map)
          
      print "pose of closest pregrasp position in /map"
      print closest_one

      #print userdata.list_of_target_positions[userdata.list_of_id_for_target_positions.index(min_id)].pre_grasp.pose
          
      # BUG!!! 
          
      if min_dist > 0.2:
        
        rospy.logerr("Distance to pregrasp position too long (%fm)",min_dist)
        min_id = -1
        return 'not_completed'
      
      else:
        
        rospy.loginfo("ID of closest position is %d and distance is %f",min_id,min_dist)
      
      userdata.id_of_the_reached_position = min_id
      return 'completed'
    
    if result.timeout:
      rospy.loginfo('action timeouted')
      userdata.id_of_the_reached_position = -1
      return 'not_completed'
  
    if result.repeat:
      rospy.loginfo('request for repeating action')
      userdata.id_of_the_reached_position = -1
      return 'repeat'
    
    if result.failed:
      rospy.loginfo('Oou... Action failed')
      userdata.id_of_the_reached_position = -1
      return 'failed'
    
    rospy.loginfo('Ups... Unknown state of action')
    userdata.id_of_the_reached_position = -1
    return 'failed'

    

class move_arm_from_a_given_position_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'])
    
   
    but_gui_ns = '/but_arm_manip'
    self.action_name = but_gui_ns + '/manual_arm_manip_action'
    
  def execute(self,userdata):
    
    rospy.loginfo('Executing state move_arm_from_a_given_position_assisted')
    
    client = actionlib.SimpleActionClient(self.action_name,ManualArmManipAction)
    
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()
    
    goal = ManualArmManipGoal()
    #goal.away = True
    
    goal.allow_repeat = False
    goal.action = "Move arm to safe position"
    goal.object_name = ""
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    
    rospy.loginfo("Waiting for result")
    client.wait_for_result()
    rospy.loginfo("I have result!! :-)")
    
    result = client.get_result()
    
    rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())
    
    if result.success:
      rospy.loginfo('Hooray, successful action')
      return 'completed'
    
    if result.timeout:
      rospy.loginfo('action timeouted')
      return 'not_completed'
    
    if result.failed:
      rospy.loginfo('Oou... Action failed')
      return 'failed'
    
    rospy.loginfo('Ups... Unknown state of action')
    return 'failed'
