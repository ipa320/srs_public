#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_arm_navigation')
import rospy
import smach
import smach_ros
import actionlib
from srs_arm_navigation.msg import *
from srs_env_model.srv import AddObjectWithBoundingBox
from srs_env_model.srv import RemovePrimitive
from srs_env_model.srv import SetGraspingPosition
from srs_env_model.srv import RemoveGraspingPosition
from math import fabs
from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from srs_object_database_msgs.srv import GetMesh
from srs_object_database_msgs.srv import GetObjectId
#from srs_object_database.srv import GetMesh
#from srs_object_database.srv import GetObjectId
from arm_navigation_msgs.msg import CollisionObject
from arm_navigation_msgs.msg import CollisionObjectOperation
from arm_navigation_msgs.msg import Shape
from geometry_msgs.msg import Pose
import threading

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
    

class move_arm_to_given_positions_assisted(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'],
                         input_keys=['list_of_target_positions','list_of_id_for_target_positions','name_of_the_target_object','pose_of_the_target_object','bb_of_the_target_object'],
                         output_keys=['id_of_the_reached_position'])
   
      
  def add_grpos(self,userdata):
    
    if len(userdata.list_of_target_positions)==0:
      
      rospy.logerr('There are NO grasping target positions')
      return None
    
    if len(userdata.list_of_target_positions) != len(userdata.list_of_id_for_target_positions):
      
      rospy.logerr('List with gr. positions has different length than list with IDs')
      return None
    
    s_set_gr_pos = '/but_gui/set_grasping_position'
    rospy.loginfo("Waiting for %s service",s_set_gr_pos)
    rospy.wait_for_service(s_set_gr_pos)
    set_gr_pos = rospy.ServiceProxy(s_set_gr_pos, SetGraspingPosition)
    rospy.loginfo('Calling service %s',s_set_gr_pos)
    
    
    for idx in range(0,len(userdata.list_of_id_for_target_positions)):
    
      try:
        
        vec = Vector3()
        
        vec.x = userdata.list_of_target_positions[idx].position.x
        vec.y = userdata.list_of_target_positions[idx].position.y
        vec.z = userdata.list_of_target_positions[idx].position.z
        
        rospy.loginfo('Adding gr pos id %d [x=%f,y=%f,z=%f]',userdata.list_of_id_for_target_positions[idx],vec.x,vec.y,vec.z)
        
        res = set_gr_pos(name=userdata.name_of_the_target_object,
                         pos_id=idx,
                         position=vec)
        
      except Exception, e:
        
        rospy.logerr('Cannot add gr pos IM for pos ID: %d, error: %s',idx,str(e))
    
    
    
  def add_im(self,userdata):
    
    
    s_get_object_id = '/get_models'
    rospy.loginfo("Waiting for %s service",s_get_object_id)
    rospy.wait_for_service(s_get_object_id)
    get_object_id = rospy.ServiceProxy(s_get_object_id, GetObjectId)
    rospy.loginfo('Calling service %s',s_get_object_id)
  
    obj_db_id = None
  
    try:
  
      res = get_object_id(type=userdata.name_of_the_target_object)
      
      obj_db_id = int(res.model_ids[0])
      
      rospy.loginfo('Object name (%s) successfully converted to ID (%d)',userdata.name_of_the_target_object,obj_db_id)
    
    except Exception, e:
    
      rospy.logerr('Error on converting name (%s) to ID... Lets use ID=1. Error: %s',userdata.name_of_the_target_object,str(e))
      obj_db_id = 1
    
    shape = None
    mesh = None
    db_shape = None
    
    s_get_model_mesh = '/get_model_mesh'
    rospy.loginfo("Waiting for %s service",s_get_model_mesh)
    rospy.wait_for_service(s_get_model_mesh)
    get_model_mesh = rospy.ServiceProxy(s_get_model_mesh, GetMesh)
    rospy.loginfo('Calling service %s (with ID=%d)',s_get_model_mesh,obj_db_id)
    
    try:
      
      object_shape = get_model_mesh(model_ids=[obj_db_id])
      shape = object_shape.msg[0].mesh
      #db_shape = shape
      
    except Exception, e:
      
      rospy.logerr('Cannot get mesh from db. We will use default one for milkbox. Error: %s',str(e))
      mesh = 'package://cob_gazebo_objects/Media/models/milk.dae'
      
    
    s_add_object = 'but_gui/add_object_with_bounding_box'
    
    rospy.loginfo("Waiting for %s service",s_add_object)
    rospy.wait_for_service(s_add_object)
    add_object = rospy.ServiceProxy('but_gui/add_object_with_bounding_box', AddObjectWithBoundingBox)
    rospy.loginfo('Calling %s service',s_add_object)
    
    #print 'POSE'
    #print userdata.pose_of_the_target_object.pose
    
    #print 'BB_MIN'
    #print userdata.bb_of_the_target_object['bb_min']
    
    #print 'BB_MAX'
    #print userdata.bb_of_the_target_object['bb_max']
    
    # position and orientation
    bpose = userdata.pose_of_the_target_object.pose

    # calculation of bounding box scale
#    bscale = Vector3()
#    bscale.x = fabs(userdata.bb_of_the_target_object['bb_min'].x - userdata.bb_of_the_target_object['bb_max'].x)
#    bscale.y = fabs(userdata.bb_of_the_target_object['bb_min'].y - userdata.bb_of_the_target_object['bb_max'].y)
#    bscale.z = fabs(userdata.bb_of_the_target_object['bb_min'].z - userdata.bb_of_the_target_object['bb_max'].z)

    # color of the bounding box
    color = ColorRGBA()
    color.r = 1
    color.g = 1
    color.b = 0
    color.a = 1
    
    try:
      
      add_object(frame_id = '/base_link',
                 name = userdata.name_of_the_target_object,
                 description = 'Object to grasp',
                 pose = bpose,
#                 scale = bscale,
                 bounding_box_lwh = userdata.bb_of_the_target_object['bb_lwh'],
#                 bounding_box_min = userdata.bb_of_the_target_object['bb_min'],
#                 bounding_box_max = userdata.bb_of_the_target_object['bb_max'],
                 color = color,
                 resource = mesh,
                 shape = shape,
                 use_material = True)
    except Exception, e:
      
      rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
      
    return shape
    
    
  def remove_im(self,userdata):
    
    # clean-up : removing interactive marker...
    s_remove_object = 'but_gui/remove_primitive'
    rospy.loginfo("Waiting for %s service",s_remove_object)
    rospy.wait_for_service(s_remove_object)
    remove_object = rospy.ServiceProxy(s_remove_object, RemovePrimitive)
    
    try:
    
      remove_object(name=userdata.name_of_the_target_object)
      
    except Exception, e:
      
      rospy.logerr('Cannot remove IM object from the scene, error: %s',str(e))
  
  def execute(self,userdata):
        
    rospy.loginfo('Executing state move_arm_to_given_positions_assisted (%s)',userdata.name_of_the_target_object)    
       
    db_shape = None
    
    # add IM to the scene
    db_shape = self.add_im(userdata)
    
    # add IM for grasping positions
    self.add_grpos(userdata)
    
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
    client = actionlib.SimpleActionClient('manual_arm_manip_action',ManualArmManipAction)
    
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
    
  def execute(self,userdata):
    
    rospy.loginfo('Executing state move_arm_from_a_given_position_assisted')
    
    client = actionlib.SimpleActionClient('manual_arm_manip_action',ManualArmManipAction)
    
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
