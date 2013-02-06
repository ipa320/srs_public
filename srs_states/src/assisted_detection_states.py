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

from srs_symbolic_grounding.srv import *
#from srs_symbolic_grounding.msg import *

#from srs_grasping.srv import *
from geometry_msgs.msg import *

from sensor_msgs.msg import *
from srs_assisted_detection.srv import *
from geometry_msgs.msg import *
## Detect state
#
# outcomes save the outcome from the sm from the service function
outcome_detectObjectSrv  = ''
outcome_user_intervention = ''
#flag will be set if the service was called
assisted_detection_service_called=False
user_intervention_service_called=0


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
        if (assisted_detection_service_called):
            userdata.object_list=self.object_list
            return outcome_detectObjectSrv 
    
    def detectObjectSrv(self,req):
       
       
       
        global outcome_detectObjectSrv 
       
        if self.preempt_requested():
           self.service_preempt()
           outcome_detectObjectSrv = 'preempted'
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
                
                outcome_detectObjectSrv = 'preempted'
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
            outcome_detectObjectSrv = 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_object_detection, DetectObjects)
            req = DetectObjectsRequest()
            req.object_name.data = self.object_name
            res = detector_service(req)
            self.object_list=res.object_list
            
            
          

            outcome_detectObjectSrv = 'succeeded'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            outcome_detectObjectSrv = 'failed'

        detector_response=UiDetectorResponse()
        if len(res.object_list.detections) > 0:
            detector_response.object_list.header=res.object_list.header
            for x in range(len(res.object_list.detections)):
                detector_response.object_list.detections.insert(x,res.object_list.detections[x])
        
        #shutdown server and set flag true        
        global assisted_detection_service_called
        assisted_detection_service_called=True
        s.shutdown()
        
        for item in detector_response.object_list.detections:
            item.pose.header.stamp = rospy.Time.now()
        
        
        
        self.object_list=detector_response.object_list
        #return service answer
        return detector_response  
    


       


     



class user_intervention_on_detection(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'bb_move', 'give_up', 'failed', 'preempted', 'retry'],
                             input_keys = ['target_object_name', 'target_object_list'],
                             output_keys=['object','object_pose','bb_pose'])
        
        self.target_object_name=''
        self.object_list=UiDetectorResponse()
        
        self.object=Detection()
        self.object_pose=Pose()
        self.bb_pose=Pose2D()

                
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        self.target_object_name=userdata.target_object_name
        
        self.object_list=userdata.target_object_list
        global s
        global s2
        
        s = rospy.Service('assisted_answer', UiAnswer, self.answerObjectSrv)
        s2 = rospy.Service('assisted_BBmove', BBMove, self.moveBBSrv)

        rospy.loginfo("Assisted Answer ready.")
        s.spin()
        s2.spin()
        if(user_intervention_service_called==1):
            #userdata.object=self.object
            #userdata.object_pose=self.object_pose
            #userdata.bb_pose=self.bbpose
            
            #this part code should be replaced by the user selection
            # select nearest object in x-y-plane in head_camera_left_link
            min_dist = 2 # start value in m
            obj = Detection()
            for item in userdata.target_object_list.detections:
                dist = sqrt(item.pose.pose.position.x*item.pose.pose.position.x+item.pose.pose.position.y*item.pose.pose.position.y)
                if dist < min_dist:
                    min_dist = dist
                    obj = copy.deepcopy(item)
            
            
            global listener
            try:
                #transform object_pose into base_link
                object_pose_in = obj
                
                print object_pose_in
                
                #object_pose_in.header.stamp = listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
                object_pose_in.header.stamp = rospy.Time.now()
                object_pose_map = listener.transformPose("/map", object_pose_in)
            except rospy.ROSException, e:
                print "Transformation not possible: %s"%e
                return 'failed'
            
            
            userdata.object_pose=object_pose_map
            userdata.object=self.object
            
            return outcome_user_intervention
        if(user_intervention_service_called==2):
            print self.bb_pose
            userdata.bb_pose=[self.bb_pose.x,self.bb_pose.y,self.bb_pose.theta]
            return outcome_user_intervention

        
        
    def answerObjectSrv(self,req):    
        global user_intervention_service_called
        global outcome_user_intervention
        
        user_intervention_service_called=1
        
        rospy.loginfo("Get Object information")
        answer=UiAnswerResponse()
        
        rospy.loginfo("%s", req.action)
        
        if(req.action=='give up'):
            outcome_user_intervention = 'give up'
            answer.message.data='give up, process stopped'

        #save
        elif(req.action=='succeeded'):
            #get position from good object
            pose=Pose()
            pose.position.x=self.object_list.detections[req.id].pose.pose.position.x
            pose.position.y=self.object_list.detections[req.id].pose.pose.position.y
            pose.position.z=self.object_list.detections[req.id].pose.pose.position.z
            pose.orientation.x=self.object_list.detections[req.id].pose.pose.orientation.x
            pose.orientation.y=self.object_list.detections[req.id].pose.pose.orientation.y
            pose.orientation.z=self.object_list.detections[req.id].pose.pose.orientation.z
            pose.orientation.w=self.object_list.detections[req.id].pose.pose.orientation.w
        
            print pose
            self.object_pose=pose
            self.object=self.object_list.detections[req.id]
            
            outcome_user_intervention = 'succeeded'
            answer.message.data='succeeded, go to next step'
        
        else: #retry detection
            outcome_user_intervention = 'retry'
            answer.message.data='retry, re-detect the object'
 

        #shutdown both service
        s.shutdown()
        s2.shutdown()
        return answer
        
    def moveBBSrv(self,req):
        rospy.loginfo("Get BB information")
        global user_intervention_service_called
        global outcome_user_intervention
        
        user_intervention_service_called=2
        #BBmove service base and then movement
        moveBB=BBMoveResponse()
        
        try:
            rospy.wait_for_service('scan_base_pose',10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            s.shutdown()
            s2.shutdown()
            moveBB.message.data='service failed try again'

            outcome_detectObjectSrv = 'failed'
        
        try:
            base_pose_service = rospy.ServiceProxy('scan_base_pose', ScanBasePose)
            req_scan = ScanBasePoseRequest()
            srs_info=SRSSpatialInfo()
            srs_info.l=req.l
            srs_info.w=req.w
            srs_info.h=req.h
            srs_info.pose=req.pose 
            req_scan.parent_obj_geometry=srs_info
            res = base_pose_service(req_scan)
            self.bb_pose=res.scan_base_pose_list[0]
            #print res
            s.shutdown()
            s2.shutdown()
            moveBB.message.data='moving to better position'

            

            outcome_user_intervention = 'bb_move'
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            s.shutdown()
            s2.shutdown()
            moveBB.message.data='service failed try again'

            outcome_user_intervention = 'failed'
        
        #service call to get better position 
        #pose=Pose()
       # pose.position.x=1
        #pose.position.y=2
        #pose.position.z=3
        #self.bb_pose=pose
        
       # outcome_user_intervention='bb_move'

        #shutdown both service

        return moveBB
