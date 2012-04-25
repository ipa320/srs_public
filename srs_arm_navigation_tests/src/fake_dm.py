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
from arm_manip_generic_states import *
from cob_object_detection_msgs.srv import DetectObjects
import std_msgs.msg
import sensor_msgs.msg
from srs_knowledge.srv import GetObjectsOnMap 
from geometry_msgs.msg import Pose
from srs_grasping.srv import GetGraspsFromPosition
from tf import TransformListener
from numpy import *
import sys
from srs_object_database_msgs.srv import GetObjectId
#from srs_object_database_msgs.srv import GetObjectId

def main():
  
  rospy.init_node('fake_dm', log_level=rospy.DEBUG)
  rospy.loginfo("Script for testing but arm manual manipulation")
  tfl = TransformListener()
  
  # sm_to = move_arm_to_given_positions_assisted()
  
  sm = smach.StateMachine(outcomes=['fdm_completed','fdm_not_completed','fdm_failed','fdm_pre-empted'],
                             input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object',
                                         'pose_of_the_target_object','bb_of_the_target_object'],
                             output_keys=['id_of_the_reached_position'])
  
  sm.userdata.name_of_the_target_object = 'MilkBox0'
  
  rospy.loginfo('Waiting for object_detect service')
  rospy.wait_for_service('/object_detection/detect_object')
    
  detect_objects = rospy.ServiceProxy('/object_detection/detect_object',DetectObjects)
    
  obj_str = std_msgs.msg.String('milk')
  roi_msg = sensor_msgs.msg.RegionOfInterest()          
    
  obj = None
    
  try:
    obj = detect_objects(object_name=obj_str,roi=roi_msg)
  except Exception, e:
    
    rospy.logerr("Error on calling fake det. service: %s",str(e))
    sys.exit(0)
    
  # test if there is any detected object
  if len(obj.object_list.detections)==0:
    
    rospy.logerr('Detector cannot detect any object...')
    sys.exit(0)
    
    
  object = obj.object_list.detections[0]
  
  # lets convert obj string to ID
  get_objects_on_map = rospy.ServiceProxy('/get_objects_on_map',GetObjectsOnMap)
  
  res_obj = None
  
  try:
    
    res_obj = get_objects_on_map('ipa-kitchen-map.owl',True)
    
  except Exception, e:
    
    rospy.logerr("Error on calling knowledge_db service: %s",str(e))
    sys.exit(0)
    
  
  # TODO test jestli je to integer !!!!!
  obj_idx = res_obj.objects.index(sm.userdata.name_of_the_target_object)
  
  
  
  # yes, we have object ID!!!! :-D
  obj_id = res_obj.houseHoldId[obj_idx]
  rospy.loginfo('Fine, ID for %s is %s',sm.userdata.name_of_the_target_object,obj_id)
  
  # ok, get grasps...
  rospy.loginfo('Waiting for /get_grasps_from_position service')
  get_grasps = rospy.ServiceProxy('/get_grasps_from_position',GetGraspsFromPosition)
    
  print "obj_position"
  print object.pose

  transf_target = '/base_link'
  
  rospy.loginfo('Lets transform pose from %s to %s frame',object.pose.header.frame_id,transf_target)
  
  if not tfl.frameExists(object.pose.header.frame_id):
    rospy.logerr('Frame %s does not exist',object.pose.header.frame_id)
    sys.exit(0)
  
  if not tfl.frameExists(transf_target):
    rospy.logerr('Frame %s does not exist',transf_target)
    sys.exit(0) 
    

  t = rospy.Time(0)
  
  rospy.loginfo('Waiting for transform for some time...')
  tfl.waitForTransform(transf_target,object.pose.header.frame_id,t,rospy.Duration(5))
  
  if tfl.canTransform(transf_target,object.pose.header.frame_id,t):
    
    obj_pose_transf = tfl.transformPose(transf_target,object.pose)
    
  else:
    
    rospy.logerr('Transformation is not possible!')
    sys.exit(0)
      
  
  print "obj_position - transformed"
  print obj_pose_transf
  
  # TODO conversion of ID using rosservice call /get_models '{type : "9"}' ---- int(obj_id)
  
  rospy.loginfo('We will try to convert knowledge_db ID (%s) to object_db ID.',int(obj_id))
  
  rospy.wait_for_service('/get_models')
  convert_id = rospy.ServiceProxy('/get_models',GetObjectId)
  
  objdb_id = None
  
  try:
  
    cres = convert_id(type=str(obj_id))
    objdb_id = int(cres.model_ids[0])
    rospy.loginfo('OBJDB ID=%d, model_desc=%s, model_category=%s',objdb_id,cres.model_desc[0],cres.model_category[0])
    
  except Exception, e:
    
    rospy.logerr('Error on converting ID (we will use ID=1): %s',str(e))
    objdb_id = 1
    
  
  rospy.loginfo('We will try to find some grasping positions, be patient')

  grasps_res = None


  fake_pregrasp_positions = rospy.get_param('~fake_pregrasp_positions', True)
  
  if not fake_pregrasp_positions:
  
    try:
    
        grasps_res = get_grasps(object_id=objdb_id,object_pose=obj_pose_transf.pose)
        
        if grasps_res.feasible_grasp_available == False:
        
          rospy.logerr('Feasible grasp for object ID %s is not available :-( (lets use fake positions...)',int(obj_id))
          grasp_res = None
          
        else:
          
          rospy.loginfo('Hooray, we are able to grasp! ;)')
        
    except Exception, e:
        
        rospy.logerr('Error on getting grasps: %s',str(e))
        grasps_res = None


  
  # test if variable is defined
  if grasps_res is not None:
      
      try:
          
           grasps_res.grasp_configuration
           
      except NameError:
      
         grasps_res.grasp_configuration = None
     
      
    
  if (grasps_res is None) or (fake_pregrasp_positions):
      
      if grasps_res is None:
        
        rospy.logerr('Problem with pre-grasp positions. We will use fake ones...')
        
      if fake_pregrasp_positions:
        
         rospy.logerr('Ok. Lets use fake pregrasp positions (required by parameter)')
      
      sm.userdata.list_of_id_for_target_positions = [5,8,13,72]
      
      fake_grasp_positions = [Pose(),Pose(),Pose(),Pose()]
  
      # relative position to object
      fake_grasp_positions[0].position.x = -0.2
      fake_grasp_positions[0].position.y = 0
      fake_grasp_positions[0].position.z = 0.1
      fake_grasp_positions[0].orientation.x = 0
      fake_grasp_positions[0].orientation.y = 0
      fake_grasp_positions[0].orientation.z = 0
      fake_grasp_positions[0].orientation.w = 1 
          
      fake_grasp_positions[1].position.x = 0.2
      fake_grasp_positions[1].position.y = 0
      fake_grasp_positions[1].position.z = 0.1
      fake_grasp_positions[1].orientation.x = 0
      fake_grasp_positions[1].orientation.y = 0
      fake_grasp_positions[1].orientation.z = 0
      fake_grasp_positions[1].orientation.w = 1 
          
      fake_grasp_positions[2].position.x = 0
      fake_grasp_positions[2].position.y = -0.2
      fake_grasp_positions[2].position.z = 0.1
      fake_grasp_positions[2].orientation.x = 0
      fake_grasp_positions[2].orientation.y = 0
      fake_grasp_positions[2].orientation.z = 0
      fake_grasp_positions[2].orientation.w = 1 
          
      fake_grasp_positions[3].position.x = 0
      fake_grasp_positions[3].position.y = 0.2 
      fake_grasp_positions[3].position.z = 0.1
      fake_grasp_positions[3].orientation.x = 0
      fake_grasp_positions[3].orientation.y = 0
      fake_grasp_positions[3].orientation.z = 0
      fake_grasp_positions[3].orientation.w = 1 
      
      sm.userdata.list_of_target_positions = fake_grasp_positions
      
  else:
    
      rospy.loginfo('We will use real pre-gr positions') 
      
      pre_gr = list()    
      sm.userdata.list_of_id_for_target_positions = list()
      idx = 100

      for it in grasps_res.grasp_configuration:
  
        pre_gr.append(it.pre_grasp)
        sm.userdata.list_of_id_for_target_positions.append(idx)
        idx = idx+1
      
      sm.userdata.list_of_target_positions = pre_gr
      
  
  
  sm.userdata.pose_of_the_target_object = obj_pose_transf
  sm.userdata.bb_of_the_target_object = {'bb_lwh': object.bounding_box_lwh}
  
  with sm:
    
    smach.StateMachine.add('to', move_arm_to_given_positions_assisted(),
                           transitions={'completed':'from','not_completed':'from','pre-empted':'from','failed':'from'},
                           remapping={'list_of_target_positions':'list_of_target_positions',
                                      'list_of_id_for_target_positions':'list_of_id_for_target_positions',
                                      'name_of_the_target_object':'name_of_the_target_object',
                                         'pose_of_the_target_object':'pose_of_the_target_object',
                                         'bb_of_the_target_object':'bb_of_the_target_object'}) 
  
    smach.StateMachine.add('from',move_arm_from_a_given_position_assisted(),
                           transitions={'completed':'fdm_completed','not_completed':'fdm_not_completed','pre-empted':'fdm_pre-empted','failed':'fdm_failed'})
    
  rospy.loginfo('Executing state machine...') 
  output = sm.execute()
  
  rospy.loginfo("ID of the reached position: %d",sm.userdata.id_of_the_reached_position)


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException: pass
  
