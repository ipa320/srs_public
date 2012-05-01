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
    def __init__(self,object_name = ""):
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
                
        global s
        rospy.loginfo("Assisted Detection ready.")
        self.object_name=userdata.object_name
        s = rospy.Service('assisted_detection', UiDetector, self.detectObjectSrv)
        s.spin()
        if (call):
            userdata.object_list=self.object_list
            return outcome
    
    def detectObjectSrv(self,req):
       
       
       
        global outcome
       
        if self.preempt_requested():
           self.service_preempt()
           outcome= 'preempted'
           return detector_response
        
        # move sdh as feedback
        sss.move("sdh","cylclosed",False)

        # make the robot ready to inspect the scene
       
        sss.say([current_task_info.speaking_language['Search'] + self.object_name + "."],False)
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

        

        # move sdh as feedback
        sss.move("sdh","home",False)

        # wait for image to become stable
        sss.sleep(2)


        # check if object detection service is available
        try:
            rospy.wait_for_service(self.srv_name_object_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            outcome= 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
            req = DetectObjectsRequest()
            req.object_name.data = self.object_name
            res = detector_service(req)
            self.object_list=res.object_list
            
            

            outcome= 'succeeded'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            outcome= 'failed'

        detector_response=UiDetectorResponse()
        if len(res.object_list.detections) > 0:
            detector_response.object_list.header=res.object_list.header
            for x in range(len(res.object_list.detections)):
                detector_response.object_list.detections.insert(x,res.object_list.detections[x])
        global call
        call=True
        s.shutdown()
        self.object_list=detector_response.object_list
        return detector_response  
    


       


     



class user_intervention_on_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'bb_move', 'give_up', 'failed', 'preempted'],
                             input_keys = ['target_object_name', 'target_object_list'],
                             output_keys=['object','object_pose','bb_pose'])
        
        self.target_object_name=''
        self.object_list=UiDetectorResponse()
        
        self.object=Detection()
        self.object_pose=Pose()
        self.bbpose=Pose()
        
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        self.target_object_name=userdata.target_object_name
        
        self.object_list=userdata.target_object_list
        print self.object_list
        global s
        global s2
        s = rospy.Service('assisted_answer', UiAnswer, self.answerObjectSrv)
        s2 = rospy.Service('assisted_BBmove', BBMove, self.moveBBSrv)

        rospy.loginfo("Assisted answer ready.")
        s.spin()
        s2.spin()
        if(call2):
            #userdata.object=self.object
            #userdata.object_pose=self.object_pose
            #userdata.bb_pose=self.bbpose
            print self.object_pose
            global listener
            try:
                #transform object_pose into base_link
                object_pose_in = self.object.pose
                object_pose_in.header.stamp = listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
                object_pose_map = listener.transformPose("/map", object_pose_in)
            except rospy.ROSException, e:
                print "Transformation not possible: %s"%e
                return 'failed'
            
            userdata.object_pose=object_pose_map
            userdata.object=self.object
            
            return outcome2
        
    def answerObjectSrv(self,req):    
        global call2
        global outcome2
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
        pose.position.x=self.object_list.detections[req.id].pose.pose.position.x
        pose.position.y=self.object_list.detections[req.id].pose.pose.position.y
        pose.position.z=self.object_list.detections[req.id].pose.pose.position.z
        pose.orientation.x=self.object_list.detections[req.id].pose.pose.orientation.x
        pose.orientation.y=self.object_list.detections[req.id].pose.pose.orientation.y
        pose.orientation.z=self.object_list.detections[req.id].pose.pose.orientation.z
        pose.orientation.w=self.object_list.detections[req.id].pose.pose.orientation.w
        
        
        self.object_pose=pose
        self.object=self.object_list.detections[req.id]
        
       
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
        global call2
        global outcome2
        
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
        self.bbpose=pose
        
        outcome2='bb_move'
        
        s.shutdown()
        s2.shutdown()
        return moveBB
    
