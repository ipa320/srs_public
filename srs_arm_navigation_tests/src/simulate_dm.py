import roslib; roslib.load_manifest('srs_arm_navigation_tests')
import rospy
import smach
import smach_ros
import actionlib
from cob_object_detection_msgs.srv import DetectObjects
import std_msgs.msg
import sensor_msgs.msg
#from srs_knowledge.srv import GetObjectsOnMap 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
#from srs_grasping.srv import GetGraspsFromPosition
from srs_grasping.srv import GetPreGrasp
#from tf import TransformListener
from numpy import *
import sys
from srs_object_database_msgs.srv import GetObjectId
from srs_msgs.msg import GraspConfiguration
#from srs_object_database_msgs.srv import GetObjectId
from shared_state_information import *

class simulate_dm(smach.State):
  def __init__(self):
    smach.State.__init__(self,outcomes=['completed','not_completed','failed','pre-empted'],
                         output_keys=['list_of_target_positions',
                                      'list_of_id_for_target_positions',
                                      'name_of_the_target_object',
                                      'pose_of_the_target_object',
                                      'bb_of_the_target_object'])
    
    #self.knowledge_db_name = rospy.get_param('~knowledge_db_name', 'MilkBox0')
    self.object_db_name = rospy.get_param('~object_db_name', 'MilkBox')
    self.object_detector_name = rospy.get_param('~object_detector_name', 'milk')
    
    
  # detects object and performs transformation to base_link coordinates
  def detect_object(self,userdata):
    
    rospy.loginfo('Waiting for object_detect service')
    rospy.wait_for_service('/object_detection/detect_object')
      
    detect_objects = rospy.ServiceProxy('/object_detection/detect_object',DetectObjects)
      
    obj_str = std_msgs.msg.String(self.object_detector_name)
    roi_msg = sensor_msgs.msg.RegionOfInterest()          
      
    try:
      
      obj = detect_objects(object_name=obj_str,roi=roi_msg)
      
    except Exception, e:
      
      rospy.logerr("Error on calling fake det. service: %s",str(e))
      return 'failed'
      
    # test if there is any detected object
    if len(obj.object_list.detections)==0:
      
      rospy.logerr('Detector cannot detect any object...')
      return 'failed'
      
      
    object = obj.object_list.detections[0]
    
    print "obj_position"
    print object.pose
    
    self.detected_object_bb = {'bb_lwh': object.bounding_box_lwh}
  
    transf_target = '/base_link'
    
    rospy.loginfo('Lets transform pose from %s to %s frame',object.pose.header.frame_id,transf_target)
    
    if not listener.frameExists(object.pose.header.frame_id):
      rospy.logerr('Frame %s does not exist',object.pose.header.frame_id)
      return 'failed'
    
    if not listener.frameExists(transf_target):
      rospy.logerr('Frame %s does not exist',transf_target)
      return 'failed' 
      
    t = rospy.Time(0)
    #object.pose.header.stamp = rospy.Time.now() # hack.........
    
    rospy.loginfo('Waiting for transform for some time...')
    listener.waitForTransform(transf_target,object.pose.header.frame_id,t,rospy.Duration(5))
    
    if listener.canTransform(transf_target,object.pose.header.frame_id,t):
      
      obj_pose_transf = listener.transformPose(transf_target,object.pose)
      
    else:
      
      rospy.logerr('Transformation is not possible!')
      return 'failed'
        
    
    print "obj_position - transformed"
    print obj_pose_transf
    
    self.detected_object_pose = obj_pose_transf
    
  # find object db ID for given object db name
  def odb_name_to_odb_id(self,userdata):
    
    rospy.loginfo('We will try to convert object_db name (%s) to object_db ID.',self.object_db_name)
    
    rospy.wait_for_service('/get_models')
    convert_id = rospy.ServiceProxy('/get_models',GetObjectId)
    
    try:
    
      # it seems that service ignores input argument, but give it a try...
      cres = convert_id(type=self.object_db_name)

      # find which item has category equal to our object_db_name
      idx = cres.model_category.index(self.object_db_name)

      objdb_id = int(cres.model_ids[idx])
      
      rospy.loginfo('object_db ID=%d, model_desc=%s, model_category=%s',objdb_id,cres.model_desc[idx],cres.model_category[idx])
      
    except Exception, e:
      
      rospy.logerr('Error on converting object_db name to object_db ID: %s',str(e))
      return 'failed'
    
    self.odb_id = objdb_id
    
    
  def get_fake_pregrasps(self,userdata):
    
    rospy.logerr('Ok. Lets use fake pregrasp positions')
        
    userdata.list_of_id_for_target_positions = [5,8,13,72]
        
    fake_grasp_positions = [GraspConfiguration(), GraspConfiguration(), GraspConfiguration(), GraspConfiguration()]
  
    # TRANSFORM POSES FROM OBJECT COORDS INTO BASE_LINK ??
    
    ox = self.detected_object_pose.pose.position.x
    oy = self.detected_object_pose.pose.position.y
    oz = self.detected_object_pose.pose.position.z
    
    # side
    fake_grasp_positions[0].pre_grasp.pose.position.x = ox -0.25
    fake_grasp_positions[0].pre_grasp.pose.position.y = oy
    fake_grasp_positions[0].pre_grasp.pose.position.z = oz
    fake_grasp_positions[0].pre_grasp.pose.orientation.x = 0.707
    fake_grasp_positions[0].pre_grasp.pose.orientation.y = 0
    fake_grasp_positions[0].pre_grasp.pose.orientation.z = 0.707
    fake_grasp_positions[0].pre_grasp.pose.orientation.w = 0
           
    # mside 
    fake_grasp_positions[1].pre_grasp.pose.position.x = ox + 0.25
    fake_grasp_positions[1].pre_grasp.pose.position.y = oy
    fake_grasp_positions[1].pre_grasp.pose.position.z = oz
    fake_grasp_positions[1].pre_grasp.pose.orientation.x = 0.707
    fake_grasp_positions[1].pre_grasp.pose.orientation.y = 0
    fake_grasp_positions[1].pre_grasp.pose.orientation.z = -0.707
    fake_grasp_positions[1].pre_grasp.pose.orientation.w = 0
        
    #front 
    fake_grasp_positions[2].pre_grasp.pose.position.x = ox
    fake_grasp_positions[2].pre_grasp.pose.position.y = oy -0.25
    fake_grasp_positions[2].pre_grasp.pose.position.z = oz
    fake_grasp_positions[2].pre_grasp.pose.orientation.x = 0
    fake_grasp_positions[2].pre_grasp.pose.orientation.y = 0.707
    fake_grasp_positions[2].pre_grasp.pose.orientation.z = 0.707
    fake_grasp_positions[2].pre_grasp.pose.orientation.w = 0
    
    # back
    fake_grasp_positions[3].pre_grasp.pose.position.x = ox
    fake_grasp_positions[3].pre_grasp.pose.position.y = oy + 0.25
    fake_grasp_positions[3].pre_grasp.pose.position.z = oz
    fake_grasp_positions[3].pre_grasp.pose.orientation.x = 0.500543638659
    fake_grasp_positions[3].pre_grasp.pose.orientation.y = 0.486927459697
    fake_grasp_positions[3].pre_grasp.pose.orientation.z = -0.50115650651
    fake_grasp_positions[3].pre_grasp.pose.orientation.w = 0.511077167143
        
    userdata.list_of_target_positions = fake_grasp_positions
    
  def get_pregrasps(self,userdata):
    
    # ok, get grasps...
    rospy.loginfo('Waiting for /get_pregrasp service')
    #get_grasps = rospy.ServiceProxy('/get_grasps_from_position',GetGraspsFromPosition)
    get_grasps = rospy.ServiceProxy('/get_pregrasps',GetPreGrasp)
    
    rospy.loginfo('We will try to find some grasping positions, be patient')

    try:
      
        grasps_res = get_grasps(object_id=self.odb_id,
                                object_pose=self.detected_object_pose.pose,
                                num_configurations=1)
          
        print "Real pregrasp positions from srs_grasping:"
        print grasps_res
        
          
    except Exception, e:
          
        rospy.logerr('Error on getting grasps: %s',str(e))
        return 'failed'
  
  
    try:
      
      grasps_res.side
      grasps_res.mside
      grasps_res.front
      #grasps_res.back
      grasps_res.top
      #grasps_res.down
      
    except NameError:
        
      ROS_ERROR("Some name in pregrasps positions is not defined. We will use fake pregr. positions.");
      return 'failed'
       
    
    list_of_id_for_target_positions = []
    list_of_target_positions = []
    
    if len(grasps_res.side)==1:
          
      list_of_id_for_target_positions.append(1)
      list_of_target_positions.append(grasps_res.side[0])
      
    if len(grasps_res.mside)==1:
          
      list_of_id_for_target_positions.append(2)
      list_of_target_positions.append(grasps_res.mside[0])
      
    if len(grasps_res.front)==1:
          
      list_of_id_for_target_positions.append(3)
      list_of_target_positions.append(grasps_res.front[0])
      
    #if len(grasps_res.back)==1:
          
    #  list_of_id_for_target_positions.append(4)
    #  list_of_target_positions.append(grasps_res.back[0])
      
    if len(grasps_res.top)==1:
          
      list_of_id_for_target_positions.append(4)
      list_of_target_positions.append(grasps_res.top[0])
      
    #if len(grasps_res.down)==1:
          
    #  list_of_id_for_target_positions.append(6)
    #  list_of_target_positions.append(grasps_res.down[0])
      
    userdata.list_of_id_for_target_positions = list_of_id_for_target_positions
    userdata.list_of_target_positions = list_of_target_positions

  def execute(self,userdata):
      
    global listener
    
    rospy.loginfo('Preparing all stuff for move_arm_to_a_given_position_assisted state')
    
    # call object detection service
    if self.detect_object(userdata) == 'failed':
      
      return 'failed'

    if self.odb_name_to_odb_id(userdata) == 'failed':
      
      return 'failed' 
    
    
    fake_pregrasp_positions = rospy.get_param('~fake_pregrasp_positions', False)
    
    # get pregrasp positions (fake or real one)
    if fake_pregrasp_positions == True:
      
      self.get_fake_pregrasps(userdata)
      
    else:
    
      if self.get_pregrasps(userdata) == 'failed':
        
        # ok, let's use fake pregrasp positions
        self.get_fake_pregrasps(userdata)
    
    
    userdata.pose_of_the_target_object = self.detected_object_pose
    userdata.bb_of_the_target_object = self.detected_object_bb
    userdata.name_of_the_target_object = self.object_db_name
 
    return 'completed'
