import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
import smach_ros

from math import *
import copy
from struct import *

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *
from shared_state_information import *
from eval_objects import *
from cob_3d_mapping_msgs.msg import *
#import service form semantic KB
from srs_knowledge.srv import *

## Detect state
#
# This state will try to detect an object.


class detect_object(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','retry','no_more_retries','failed','preempted'],
            input_keys=['object_name'],
            output_keys=['object','object_pose'])

        self.object_list = DetectionArray()
        self.max_retries = max_retries
        self.retries = 0
        self.object_name = object_name
        self.srv_name_object_detection = '/object_detection/detect_object'
        self.torso_poses = []
        self.torso_poses.append("back_right")
        self.torso_poses.append("back_right")
        #self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back")
        self.torso_poses.append("back")
        #self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left")
        self.torso_poses.append("back_left")
        #self.torso_poses.append("back_left_extreme")
        self.the_object = ''
        self.the_object_pose = ''


    def execute(self, userdata):

        global current_task_info
	    #userdauserdata.target_object_listta.object_name = "milk"
        # if the object has been identified, and there was no base movement of grasp, then no need to detect again
        if current_task_info.get_object_identification_state() == True:
            userdata.object_pose = self.the_object_pose
            userdata.object = self.the_object
            return 'succeeded'

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        """
        # checking extra information
        if userdata.key_region == '':
            pass #normal detection
        else:
            pass #detection in bounding box specified by key_region
        """
        #add initial value to output keys
        userdata.object = ""
        userdata.object_pose=""
        
        rospy.loginfo("object_name: %s", userdata.object_name)
        
        # determine object name
        if self.object_name != "":
            object_name = self.object_name
        elif type(userdata.object_name) is str:
            object_name = userdata.object_name
        else: # this should never happen
            rospy.logerr("Invalid userdata 'object_name'")
            self.retries = 0
            return 'failed'

        # check if maximum retries reached
        if self.retries > self.max_retries:
            self.retries = 0
            hanuserdata.target_object_listdle_torso = sss.move("torso","home",False)
            handle_torso.wait()
            return 'no_more_retries'

        # move sdh as feedback
        sss.move("sdh","cylclosed",False)

        # make the robot ready to inspect the scene
        if self.retries == 0: # only move arm, sdh and head for the first try
            rospy.loginfo ("object_name: %s", object_name)
            sss.say([current_task_info.speaking_language['Search'] + object_name + "."],False)
            handle_arm = sss.move("arm","folded-to-look_at_table",False)
            handle_torso = sss.move("torso","shake",False)
            handle_head = sss.move("head","back",False)

            if self.preempt_requested():
                self.service_preempt()
                #handle_base.set_failed(4)
                handle_arm.client.cancel_goal()
                handle_torso.client.cancel_goal()
                handle_head.client.cancel_goal()
                return 'preempted'
            else:
                handle_arm.wait()
                handle_head.wait()
                handle_torso.wait()

        handle_torso = sss.move("torso",self.torso_poses[self.retries % len(self.torso_poses)]) # have an other viewing point for each retry

        # move sdh as feedback
        sss.move("sdh","home",False)

        # wait for image to become stable
        sss.sleep(2)


        # check if object detection service is available
        try:
            rospy.wait_for_service(self.srv_name_object_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            self.retries = 0 # no object found within min_dist start value
            return 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
            req = DetectObjectsRequest()
            req.object_name.data = object_name
            res = detector_service(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            self.retries = 0
            return 'failed'

        # check for no objects
        if len(res.object_list.detections) <= 0:
            rospy.logerr("No objects found")
            self.retries += 1
            return 'retry'

        # select nearest object in x-y-plane in head_camera_left_link
        min_dist = 2 # start value in m
        obj = Detection()
        for item in res.object_list.detections:
            dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
            if dist < min_dist:
                min_dist = dist
                obj = copy.deepcopy(item)

        # check if an object could be found within the min_dist start value
        if obj.label == "":
            rospy.logerr("Object not within target range")
            self.retries += 1
            return 'retry'

        #check if label of object fits to requested object_name
        if obj.label != object_name:
            rospy.logerr("The object name doesn't fit.")
            self.retries += 1
            return 'retry'

        # we succeeded to detect an object
        userdata.object = obj
        self.the_object = obj
        object_pose_map = PoseStamped()
        self.retries = 0

        global listener

        try:
            #transform object_pose into base_link
            object_pose_in = copy.deepcopy(obj.pose)
            print "###obj", obj
            print "###obj.pose", obj.pose
            object_pose_in.header.stamp = listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
            object_pose_map = listener.transformPose("/map", object_pose_in)
            obj.pose = copy.deepcopy(object_pose_map)
            userdata.object = obj
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'

        #print object_pose_map

        userdata.object_pose=object_pose_map
        self.the_object_pose=object_pose_map

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # remember the object has been identified, if there is no base movement of grasp, no need to detect again
        current_task_info.set_object_identification_state(True)
        return 'succeeded'


"""Verifies whether the object expected at target_object_pose is actually there.
If yes, verfified_target_object_pose is returned."""
class VerifyObject(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['object_id','target_object_pose'],
      output_keys=['verfified_target_object_pose'])
    self.eo = EvalObjects()
    #self.client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)

  def execute(self, userdata):
    #object_to_search =self.eo.semantics_db_get_object_info(userdata.object_id)
    #print "Searching for object at " + str(object_to_search.objectPose.position.x) + ", " + str(object_to_search.objectPose.position.y)
    """TODO: get classID for object_id or have class_id as input"""
    object_list_map = self.eo.map_list_objects(1)#object_to_search.classID)
    #if object_to_search.classID == 1: #table
    verified_table = self.eo.verify_table(userdata.target_object_pose, object_list_map)
    if verified_table:
        userdata.verfified_target_object_pose = verified_table.pose.pose
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " found at " + str(verified_table.pose.pose.position.x) + "," + str(verified_table.pose.pose.position.y)
        return 'succeeded'
    else:
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " not found"
        return 'not_completed'
    #else:
    #  print 'Object class not supported'
    #  return 'failed'
    
"""Verifies whether the object expected at target_object_pose is actually there.
If yes, verfified_target_object_pose is returned."""
class VerifyObjectByName(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['object_name'],
      output_keys=['verfified_target_object_pose'])
    self.eo = EvalObjects()
    #self.client = actionlib.SimpleActionClient('trigger_mapping', TriggerMappingAction)

  def execute(self, userdata):
      
    target_object_pose =''
    object_id = -1000
    all_workspaces_on_map = ''
    index_of_the_target_workspace = ''

    try:
        getWorkspace = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
        all_workspaces_on_map = getWorkspace(os.environ['ROBOT_ENV'], True)
        
        #get the index of the target workspace e.g. table0    
        index_of_the_target_workspace = all_workspaces_on_map.objects.index(userdata.object_name)
        #get the pose of the workspace from knowledge service
        target_object_pose = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].pose
        #get the houseHoldID of the workspace 
        object_id = all_workspaces_on_map.houseHoldId[index_of_the_target_workspace]
        
        rospy.loginfo ("target name: %s", userdata.object_name)      
        rospy.loginfo ("target pose: %s", target_object_pose)
        rospy.loginfo ("target id: %s", object_id)

        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e 
        return 'failed'   
    
    """
    #checking object id in HHDB, maybe required for more complicated verification
    if object_id == -1000:
        #the object is not available in the HH database
        #verification not possible
        return 'not_completed'
    """
      
    #object_to_search =self.eo.semantics_db_get_object_info(userdata.object_id)
    #print "Searching for object at " + str(object_to_search.objectPose.position.x) + ", " + str(object_to_search.objectPose.position.y)
    """TODO: get classID for object_id or have class_id as input"""
    object_list_map = self.eo.map_list_objects(1)#object_to_search.classID)
    print "object list map",object_list_map
    #if object_to_search.classID == 1: #table
    verified_table = self.eo.verify_table(target_object_pose, object_list_map)
    if verified_table:
        userdata.verfified_target_object_pose = verified_table.pose.pose
        print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " found at " + str(verified_table.pose.pose.position.x) + "," + str(verified_table.pose.pose.position.y)
        return 'succeeded'
    else:
        print "table " + str(target_object_pose.position.x) + "," + str(target_object_pose.position.y) + " not found"
        return 'not_completed'
    #else:
    #  print 'Object class not supported'
    #  return 'failed'

"""Observes the space over a table and returns objects as bounding boxes"""
class GetTableObjectCluster(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['target_table_pose'],
      output_keys=['object_cluster'])
    self.eo = EvalObjects()
    self.client = actionlib.SimpleActionClient('table_object_cluster', TableObjectClusterAction)

  def execute(self, userdata):
    #object_to_search =self.eo.semantics_db_get_object_info(userdata.object_id)
    #print "Searching for object at " + str(object_to_search.objectPose.position.x) + ", " + str(object_to_search.objectPose.position.y)
    object_list_map = self.eo.map_list_objects(1)#object_to_search.classID)
    #if object_to_search.classID == 1: #table
    verified_table = self.eo.verify_table(userdata.target_table_pose, object_list_map)
    #verified_table = object_list_map.objects.shapes[2]
    if verified_table:
        hull = verified_table.points[0]
        #print verified_table.params
        #print "table " + str(userdata.target_object_pose.position.x) + "," + str(userdata.target_object_pose.position.y) + " found at " + str(verified_table.params[4]) + "," + str(verified_table.params[5])
    else:
        print "table " + str(userdata.target_table_pose.position.x) + "," + str(userdata.target_table_pose.position.y) + " not found"
        return 'not_completed'
    goal = TableObjectClusterGoal(hull)
    if not self.client.wait_for_server():#rospy.Duration.from_sec(5.0)):
      rospy.logerr('server not available')
      return 'not_completed'
    self.client.send_goal(goal)
    if not self.client.wait_for_result():#rospy.Duration.from_sec(5.0)):
      return 'not_completed'
    userdata.object_cluster = self.client.get_result().bounding_boxes
    return 'succeeded'


"""Checks, whether target_object_pose_on_table is occupied by at least one object of object_cluster.
Returns succeded if the position is free and failed if the position is occupied"""
class CheckPoseOnTableFree(smach.State):

  def __init__(self):

    smach.State.__init__(
      self,
      outcomes=['succeeded', 'failed', 'not_completed', 'preempted'],
      input_keys=['target_object_pose_on_table','object_cluster'],
      output_keys=[])

  def execute(self, userdata):
    for bb in userdata.object_cluster:
      pt1 = []
      for i in range(0,3):
        pt1.append(unpack("f",bb.data[4*i:4*i+4])[0])
      pt2 = []
      for i in range(4,7):
        pt2.append(unpack("f",bb.data[4*i:4*i+4])[0])
      x = userdata.target_object_pose_on_table.position.x
      y = userdata.target_object_pose_on_table.position.y
      if x>pt1[0] and x<pt2[0] and y>pt1[1] and y<pt2[1]:
        print "target position occupied"
        return 'failed'
      else:
        print "target position free"
    return 'succeeded'

