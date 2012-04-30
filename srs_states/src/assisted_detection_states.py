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

#from srs_grasping.srv import *
from geometry_msgs.msg import *

from sensor_msgs.msg import *
from srs_assisted_detection.srv import *
from geometry_msgs.msg import *
## Detect state
#
# This state will try to detect an object.
outcome = ''
outcome2 = ''
call=False
call2=False


class detect_object_assited(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','failed','preempted'],
            input_keys=['object_name'],
            output_keys=['object_list'])

        self.object_list = DetectionArray()
        self.object_name = object_name
        self.srv_name_object_detection = '/object_detection/detect_object'

        self.torso_poses = []
        self.torso_poses.append("back_right")
        self.torso_poses.append("back")
        self.torso_poses.append("back_left")
        self.torso_poses.append("back_right_extreme")
        self.torso_poses.append("back_extreme")
        self.torso_poses.append("back_left_extreme")
        #self.listener = tf.TransformListener()
        self.the_object = ''
        self.the_object_pose = ''


    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
                
        global s
        s = rospy.Service('assisted_detection', UiDetector, detectObjectSrv)
        rospy.loginfo("Assisted Detection ready.")
        s.spin()
        if (call):
            return outcome
    
    def detectObjectSrv(self,req):
       
        global outcome
        # move sdh as feedback
        sss.move("sdh","cylclosed",False)

        # make the robot ready to inspect the scene
        if self.retries == 0: # only move arm, sdh and head for the first try
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
                
                outcome= 'preempted'
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

            outcome= 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
            req = DetectObjectsRequest()
            req.object_name.data = userdata.object_name
            res = detector_service(req)
            userdata.object_list=res.object_list
            

            outcome= 'succeeded'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            outcome= 'failed'

        global detector_response
        if len(resp1.object_list.detections) > 0:
            detector_response.object_list.header=resp1.object_list.header
            for x in range(len(resp1.object_list.detections)):
                detector_response.object_list.detections.insert(x,resp1.object_list.detections[x])
        global call
        call=True
        s.shutdown()
        userdata.object_list=detector_response.object_list
        return detector_response  
    


       


     



class user_intervention_on_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'bb_move', 'give_up', 'failed', 'preempted'],
                             input_keys = ['target_object_name', 'object_list'],
                             output_keys=['object','object_pose','bb_pose'])
        
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        global s
        global s2
        s = rospy.Service('assisted_answer', UiAnswer, answerObjectSrv)
        s2 = rospy.Service('assisted_BBmove', BBMove, moveBBSrv)

        rospy.loginfo("Assisted answer ready.")
        s.spin()
        s2.spin()
        if(call2):
            return outcome2
        
    def answerObjectSrv(self,req):    
        call2=True
        rospy.loginfo("Get Object information")
        answer=UiAnswerResponse()
        
        if(req.action=='give up'):
            outcome2 = 'give up'
            answer.message.data='process stopped'
            return answer
            
            

        #save
        
            
       
        #get position from good object
        pose=Pose()
        pose.position.x=userdata.object_list.detections[req.id].pose.pose.position.x
        pose.position.y=userdata.object_list.detections[req.id].pose.pose.position.y
        pose.position.z=userdata.object_list.detections[req.id].pose.pose.position.z
        pose.orientation.x=userdata.object_list.detections[req.id].pose.pose.orientation.x
        pose.orientation.y=userdata.object_list.detections[req.id].pose.pose.orientation.y
        pose.orientation.z=userdata.object_list.detections[req.id].pose.pose.orientation.z
        pose.orientation.w=userdata.object_list.detections[req.id].pose.pose.orientation.w
        
        
        userdata.object_pose=pose
        userdata.object=userdata.object_list.detections[req.id]
        
       
        #global action
        #default for user
        #action='grasp'
    
        s.shutdown()
        s2.shutdown()
        outcome2 = 'succeeded'
        answer.message.data='Action is running'
        return answer
    
            
        
        
    def moveBBSrv(self,req):
        rospy.loginfo("Get BB information")
        call2=True
        #BBmove service base and then movement
        moveBB=BBMoveResponse()
        moveBB.message.data='moving to better position'
        
        #
        
        #service call to get better position 
        pose=Pose()
        pose.position.x=1
        pose.position.y=2
        pose.position.z=3
        userdata.bbpose=pose
        
        outcome2='bb_move'
        
        s.shutdown()
        s2.shutdown()
        return moveBB
    
