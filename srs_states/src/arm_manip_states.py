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

import roslib; roslib.load_manifest('srs_states')
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
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from srs_object_database_msgs.srv import GetMesh
from srs_object_database_msgs.srv import GetObjectId
from srs_object_database_msgs.srv import GetMesh
from srs_object_database_msgs.srv import GetObjectId
from arm_navigation_msgs.msg import CollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose
from tf import TransformListener
import threading
from srs_assisted_arm_navigation.srv import ArmNavCollObj
from srs_assisted_arm_navigation.srv import ArmNavMovePalmLink

class coll_obj_publisher (threading.Thread):
  
  def __init__ (self,name_of_the_target_object,frame_id,pose_of_the_target_object,shape,padding):
    
    self.name_of_the_target_object = name_of_the_target_object
    self.frame_id = frame_id
    self.pose_of_the_target_object = pose_of_the_target_object
    self.shape = shape
    self.padding = padding
    self.end = False
    threading.Thread.__init__ ( self )
  
  def run (self):
   
    coll_obj = CollisionObject()
    coll_obj.operation.operation = CollisionObjectOperation.ADD
    coll_obj.id = self.name_of_the_target_object
    coll_obj.header = rospy.Header()
    coll_obj.header.frame_id = self.frame_id
    coll_obj.poses =  [self.pose_of_the_target_object]
    coll_obj.shapes = [self.shape]
    coll_obj.padding = self.padding
    pub = rospy.Publisher('/collision_object',CollisionObject,latch=True)
    
    rospy.loginfo('Thread publishing on /collision_object topic (%s)',self.name_of_the_target_object)
    
    while (rospy.is_shutdown() == False) and (self.end == False):
      
      coll_obj.header.stamp = rospy.Time.now()
      try:
        pub.publish(coll_obj)
      except Exception, e:
        
        rospy.logerr("Error on publishing to /collision_object topic: %s", e)
        return 0
        
      rospy.logdebug('Thread is looping')
      rospy.sleep(2)
      
    return 0

# estimate the best grasp position
class assisted_arm_navigation_prepare(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'preempted'],
            input_keys=['object'],
            output_keys=['pose_of_the_target_object','bb_of_the_target_object'])
        
        self.counter = 0 


    def execute(self, userdata):
        
        """
        #object formation
        #############
        pose: 
          header: 
            seq: 0
            stamp: 
              secs: 139
              nsecs: 877000000
            frame_id: /head_color_camera_l_link
          pose: 
            position: 
              x: 0.0757232808843
              y: -0.299051730792
              z: 1.00000055144
            orientation: 
              x: 0.0246255429105
              y: 0.790639668039
              z: 0.609547053964
              w: -0.0522961467358
        bounding_box_lwh: 
          x: 0.06
          y: 0.095
          z: 0.2
        """       
        userdata.pose_of_the_target_object = userdata.object.pose
        userdata.bb_of_the_target_object = userdata.object.bounding_box_lwh
        return 'succeeded'


class move_arm_to_given_positions_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['succeeded','not_completed','failed','preempted'],
                         input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object','pose_of_the_target_object','bb_of_the_target_object'],
                         output_keys=['id_of_the_reached_position'])
    
 
    but_gui_ns = '/but_interaction_primitives'
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
    
    if len(userdata.list_of_target_positions)==0:
      
      rospy.logerr('There are NO grasping target positions')
      return None
    
    if len(userdata.list_of_target_positions) != len(userdata.list_of_id_for_target_positions):
      
      rospy.logerr('List with gr. positions has different length than list with IDs')
      return None
    
    #global s_set_gr_pos
    #s_set_gr_pos = '/but_gui/set_grasping_position'
    rospy.loginfo("Waiting for %s service",self.s_set_gr_pos)
    rospy.wait_for_service(self.s_set_gr_pos)
    set_gr_pos = rospy.ServiceProxy(self.s_set_gr_pos, SetPreGraspPosition)
    rospy.loginfo('Calling service %s',self.s_set_gr_pos)
    
    
    for idx in range(0,len(userdata.list_of_id_for_target_positions)):
    
      try:
        
        gpose = Pose()
        
        #vec = Vector3()
        #vec.x = userdata.list_of_target_positions[idx].position.x
        #vec.y = userdata.list_of_target_positions[idx].position.y
        #vec.z = userdata.list_of_target_positions[idx].position.z
        
        # we should receive it as PoseStamped
        # TODO maybe it will be necessary to do some transform of coordinates
        gpose = userdata.list_of_target_positions[idx]
        
        # shift position to be relative to detected object 
        gpose.position.x = gpose.position.x - userdata.pose_of_the_target_object.pose.position.x
        gpose.position.y = gpose.position.y - userdata.pose_of_the_target_object.pose.position.y
        gpose.position.z = gpose.position.z - userdata.pose_of_the_target_object.pose.position.z
        
        
        rospy.loginfo('Adding gr pos id %d [x=%f,y=%f,z=%f]',userdata.list_of_id_for_target_positions[idx],gpose.position.x,gpose.position.y,gpose.position.z)
        
        res = set_gr_pos(name=userdata.name_of_the_target_object,
                         pos_id=idx+1,
                         pose=gpose)
        
      except Exception, e:
        
        rospy.logerr('Cannot add gr pos IM for pos ID: %d, error: %s',idx+1,str(e))
    
    
    
  def add_im(self,userdata):
    
    tfl = TransformListener()
    
    rospy.loginfo("Waiting for %s service",self.s_get_object_id)
    rospy.wait_for_service(self.s_get_object_id)
    get_object_id = rospy.ServiceProxy(self.s_get_object_id, GetObjectId)
    rospy.loginfo('Calling service %s',self.s_get_object_id)
  
    obj_db_id = None
  
    try:
  
      res = get_object_id(type=userdata.name_of_the_target_object)
      
      obj_db_id = int(res.model_ids[0])
      
      rospy.loginfo('Object name (%s) successfully converted to ID (%d)',userdata.name_of_the_target_object,obj_db_id)
    
    except Exception, e:
    
      rospy.logerr('Error on converting name (%s) to ID... Lets use ID=1. Error: %s',userdata.name_of_the_target_object,str(e))
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
    
    # try to transform pose of detected object from /base_link to /map
    transf_target = '/map'
  
    rospy.loginfo('Lets transform pose from %s to %s frame',userdata.pose_of_the_target_object.header.frame_id,transf_target)
  
    #if not tfl.frameExists(userdata.pose_of_the_target_object.header.frame_id):
    #  rospy.logerr('Frame %s does not exist',userdata.pose_of_the_target_object.header.frame_id)
    #  sys.exit(0)
  
    #if not tfl.frameExists(transf_target):
    #  rospy.logerr('Frame %s does not exist',transf_target)
    #  sys.exit(0) 
    

    t = rospy.Time(0)
  
    rospy.loginfo('Waiting for transform for some time...')
    tfl.waitForTransform(transf_target,userdata.pose_of_the_target_object.header.frame_id,t,rospy.Duration(5))
  
    if tfl.canTransform(transf_target,userdata.pose_of_the_target_object.header.frame_id,t):
    
      obj_pose_transf = tfl.transformPose(transf_target,userdata.pose_of_the_target_object)
    
    else:
    
      rospy.logerr('Transformation is not possible!')
      sys.exit(0) 
      
    #print "Transformed pose of object"
    #print obj_pose_transf
      
      
    bpose = obj_pose_transf.pose
    #bpose = userdata.pose_of_the_target_object.pose
    
    
    try:
      
      if use_default_mesh:
      
        add_object(frame_id = transf_target,
                   name = userdata.name_of_the_target_object,
                   description = 'Object to grasp',
                   pose = bpose,
                   bounding_box_lwh = userdata.bb_of_the_target_object['bb_lwh'],
                   color = color,
                   resource = 'package://cob_gazebo_objects/Media/models/milk.dae', # TODO udelat na zaklade toho co se mi povede ziskat volani bud z mesh nebo shape...
                   #shape = shape,
                   use_material = True)
      else:
        
        add_object(frame_id = transf_target,
                   name = userdata.name_of_the_target_object,
                   description = 'Object to grasp',
                   pose = bpose,
                   bounding_box_lwh = userdata.bb_of_the_target_object['bb_lwh'],
                   color = color,
                   #resource = 'package://cob_gazebo_objects/Media/models/milk.dae', # TODO udelat na zaklade toho co se mi povede ziskat volani bud z mesh nebo shape...
                   shape = shape,
                   use_material = True)
      
      
    except Exception, e:
      
      rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
      
    return shape
    
    
  def remove_im(self,userdata):
    
    # clean-up : removing interactive marker...
    rospy.loginfo("Waiting for %s service",self.s_remove_object)
    rospy.wait_for_service(self.s_remove_object)
    remove_object = rospy.ServiceProxy(self.s_remove_object, RemovePrimitive)
    
    try:
    
      remove_object(name=userdata.name_of_the_target_object)
      
    except Exception, e:
      
      rospy.logerr('Cannot remove IM object from the scene, error: %s',str(e))
 
 
  def pregr_im_callback(self,data):
    
    rospy.loginfo("Move arm to pregrasp pos. ID=%d, object=%s",data.pos_id,data.marker_name);
    
    # TODO call service provided by arm_manip_node which will move the arm appropriately
    rospy.wait_for_service(self.s_move_arm)
    move_arm = rospy.ServiceProxy(self.s_move_arm, ArmNavMovePalmLink);
    
    try:
      
      move_arm(sdh_palm_link_pose=self.userdata.list_of_target_positions[data.pos_id]);
      
    except Exception, e:
      
      rospy.logerr('Cannot move sdh_palm_link to given position, error: %s',str(e))
    
    
  
  def execute(self,userdata):
        
    self.userdata = userdata
        
    rospy.loginfo('Executing state move_arm_to_given_positions_assisted (%s)',userdata.name_of_the_target_object)    
       
    db_shape = None
    
    # add IM to the scene
    db_shape = self.add_im(userdata)
    
    # add IM for grasping positions
    self.add_grpos(userdata)
    
    pregr_topic = "/but_interaction_primitives/" + userdata.name_of_the_target_object + "/update/move_arm_to_pregrasp"
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
    
    # ADD known object to collision map...
    #===========================================================================
    # if db_shape != None:
    # 
    # coll_obj_publisher(name_of_the_target_object = userdata.name_of_the_target_object,
    #                  frame_id = '/base_link',
    #                  pose_of_the_target_object = userdata.pose_of_the_target_object,
    #                  shape = db_shape,
    #                  padding = 0.05).start()
    # 
    # else:
    # 
    # rospy.logerr('Mesh was not obtained from db, so cannot be added to planning scene')
    #===========================================================================

    # lets start action...    
    client = actionlib.SimpleActionClient(self.action_name,ManualArmManipAction)
    
    rospy.loginfo("Waiting for actionlib server...")
    client.wait_for_server()
    
    goal = ManualArmManipGoal()
       
    
    goal.pregrasp = True
    goal.object_name = userdata.name_of_the_target_object
    goal.target_positions = userdata.list_of_target_positions
    goal.positions_ids = userdata.list_of_id_for_target_positions
    
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)
    
    rospy.loginfo("Waiting for result")
    client.wait_for_result()
    rospy.loginfo("I have result!! :-)")
    
    result = client.get_result()
    
    # clean up
    self.remove_im(userdata)
    
    #coll_obj_publisher.end = True
    
    #rospy.loginfo('Joining with thread...')
    #coll_obj_publisher.join()
    
    rospy.loginfo("Time elapsed: %ss",result.time_elapsed.to_sec())
    
    if result.success:
      rospy.loginfo('Hooray, successful action, id of reached position is: %d',result.reached_position)
      userdata.id_of_the_reached_position = result.reached_position
      return 'completed'
    
    if result.timeout:
      rospy.loginfo('action timeouted')
      userdata.id_of_the_reached_position = -1
      return 'not_completed'
    
    rospy.loginfo('action failed')
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
    goal.away = True
    
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
    
    rospy.loginfo('action failed')
    return 'failed'
