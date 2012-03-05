import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
import smach_ros

from math import *
import copy

from simple_script_server import *
sss = simple_script_server()

from cob_object_detection_msgs.msg import *
from cob_object_detection_msgs.srv import *

from shared_state_information import *



## Detect state
#
# This state will try to detect an object.


class detect_object(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','retry','no_more_retries','failed','preempted'],
            input_keys=['object_name','key_region'],
            output_keys=['object','object_pose'])

        self.object_list = DetectionArray()
        self.max_retries = max_retries
        self.retries = 0
        self.object_name = object_name
        self.srv_name_object_detection = '/object_detection/detect_object'
        
        self.torso_poses = []
        self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left_extreme")
        #self.listener = tf.TransformListener()
        self.the_object = ''
        self.the_object_pose = ''
        

    def execute(self, userdata):
        
        global current_task_info
        
        # if the object has been identified, and there was no base movement of grasp, then no need to detect again       
        if current_task_info.get_object_identification_state() == True:
            userdata.object_pose = self.the_object_pose
            userdata.object = self.the_object
            return 'succeeded'
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # checking extra information
        if userdata.key_region == '':
            pass #normal detection
        else:
            pass #detection in bounding box specified by key_region
        
        #add initial value to output keys
        userdata.object = ""
        userdata.object_pose=""
        
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
            return 'no_more_retries'
        
        # move sdh as feedback
        sss.move("sdh","cylclosed",False)
        
        # make the robot ready to inspect the scene
        if self.retries == 0: # only move arm, sdh and head for the first try
            sss.say(["I will now search for the " + object_name + "."],False)
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
            object_pose_in = obj.pose
            object_pose_in.header.stamp = listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
            object_pose_map = listener.transformPose("/map", object_pose_in)
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'
        
        print object_pose_map
        
        userdata.object_pose=object_pose_map
        self.the_object_pose=object_pose_map
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # remember the object has been identified, if there is no base movement of grasp, no need to detect again
        current_task_info.set_object_identification_state(True)
        return 'succeeded'

