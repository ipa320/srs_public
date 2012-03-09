#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_arm_navigation')
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
from srs_object_database.srv import GetObjectId

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
    rospy.logerr('Frame %s does not exist',obj_position.header.frame_id)
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
  
    cres = convert_id(type=int(obj_id))
    objdb_id = int(cres.model_ids[0])
    rospy.loginfo('OBJDB ID=%d, model_desc=%s, model_category=%s',objdb_id,cres.model_desc[0],cres.model_category[0])
    
  except Exception, e:
    
    rospy.logerr('Error on converting ID (we will use ID=1): %s',str(e))
    objdb_id = 1
    
  
#  rospy.loginfo('We will try to find some grasping positions, be patient')
#  
#  grasps_res = get_grasps(object_id=objdb_id,object_pose=obj_pose_transf.pose)
#  
#  if grasps_res.feasible_grasp_available == False:
#    rospy.logerr('Feasible grasp for object ID %s is not available :-(',int(obj_id))
#    #sys.exit(0)
#  
#  print "GRASPS"
#  print grasps_res

  # TODO prepare fake grasping positions data....
  
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
    
  # TODO add another data !!!!!!!!
  #sm.userdata.list_of_target_positions = grasps_res.grasp_configuration
  sm.userdata.list_of_target_positions = fake_grasp_positions
  sm.userdata.list_of_id_for_target_positions = [5,8,13,72]
  sm.userdata.pose_of_the_target_object = obj_pose_transf
  bbmin = Pose()
  bbmax = Pose()
  bbmin.position.x = object.pose.pose.position.x - object.bounding_box_lwh.x
  bbmin.position.y = object.pose.pose.position.y - object.bounding_box_lwh.y
#  bbmin.position.z = object.pose.pose.position.z - object.bounding_box_lwh.z
  bbmin.position.z = object.pose.pose.position.z
  bbmin.position.x = object.pose.pose.position.x + object.bounding_box_lwh.x
  bbmin.position.y = object.pose.pose.position.y + object.bounding_box_lwh.y
  bbmin.position.z = object.pose.pose.position.z + object.bounding_box_lwh.z
  sm.userdata.bb_of_the_target_object = {'bb_min': bbmin.position, 'bb_max': bbmax.position}
#  sm.userdata.bb_of_the_target_object = {'bb_min': object.pose.pose.position - object.bounding_box_lwh, 'bb_max': object.pose.pose.position + object.bounding_box_lwh}
#  sm.userdata.bb_of_the_target_object = {'bb_min': object.bounding_box_min, 'bb_max': object.bounding_box_max}
  
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
  