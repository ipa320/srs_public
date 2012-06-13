import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
import smach_ros

from math import *
import copy
from struct import *
import tf

from simple_script_server import *
sss = simple_script_server()


#from srs_grasping.srv import *
from geometry_msgs.msg import *
from cob_people_detection.srv import *
from cob_people_detection_msgs.msg import *
from srs_human_sensing.srv import *
from srs_human_sensing.msg import *

from srs_leg_detector.srv import *
from srs_body_detector.srv import *


from simple_script_server import *
sss = simple_script_server()

class leg_detection(smach.State):
    def __init__(self,object_name = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded','failed','preempted','retry'],
            input_keys=['sm_input'],
            output_keys=['pose_list','id','humans_pose','person_label'])
        
        self.srv_name_leg_detection = '/detect_legs'
        self.pose_list=[]

    def execute(self, userdata):
        rospy.loginfo("Leg detection ready.")

        if(userdata.sm_input.label==""):
                    rospy.loginfo("Searching for all peoples.")
        else:
                    rospy.loginfo("Searching for special person.")

        if self.preempt_requested():
           self.service_preempt()
           return 'preempted'
       
       
        try:
            rospy.wait_for_service(self.srv_name_leg_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            return 'failed'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_leg_detection, DetectLegs)
            req = DetectLegsRequest()
            res = detector_service(req)
            lenp=len(res.leg_list.points)
            if lenp > 3:
                lenp=3
            if lenp==0:
                return 'retry'   
            for i in range(0, lenp):    
                            
                 p=Pose2D()
                 p.x=res.leg_list.points[i].x
                 p.y=res.leg_list.points[i].y
                 p.theta=0
                 self.pose_list.append(p)
                 
            for i in range (0,len(userdata.sm_input.pose_furniture)):
                 p=Pose2D()
                 p.x=userdata.sm_input.pose_furniture[i].x
                 p.y=userdata.sm_input.pose_furniture[i].y
                 p.theta=userdata.sm_input.pose_furniture[i].theta
                 self.pose_list.append(p)

                 
            #print self.pose_list     
            if len(self.pose_list)==0:
                return 'failed'   
              
            userdata.pose_list=self.pose_list
            print len(self.pose_list)
            userdata.id=0
            userdata.humans_pose=detect_human_array()
            userdata.person_label=userdata.sm_input.label
            return 'succeeded'
        
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            return 'failed'


class move_to_better_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'failed', 'preempted'],
                             input_keys = ['pose_list', 'id','humans_pose','person_label'],
                             output_keys=['pose_list_out','id_out','humans_pose_out','person_label_out'])
        
        self.target_object_name=''
        #self.object_list
        
        
        self.object_pose=Pose()
        self.bbpose=Pose()
        self.srv_name_people_detection='/cob_people_detection/detect_people'
        
    def execute(self, userdata):
        print userdata.id
        if self.preempt_requested():
           self.service_preempt()
           return 'preempted'
       
        
        #move to better position
        #get the robot's current pose from tf
        rb_pose = Pose2D()
        listener = tf.TransformListener()
        listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(4.0)) #wait for 4secs for the coordinate to be transformed
        (trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        rb_pose.x = trans[0]
        rb_pose.y = trans[1]
        rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
        rb_pose.theta = rb_pose_rpy[2]
        rospy.sleep(0.5)
        
        x=rb_pose.x-userdata.pose_list[userdata.id].x
        y=rb_pose.y-userdata.pose_list[userdata.id].y
        
        
        #not sure about the direction
        theta=tan(x/y)       
         
        sss.move("base",[rb_pose.x,rb_pose.y,theta])
        
        userdata.id_out=userdata.id
        userdata.pose_list_out=userdata.pose_list
        userdata.humans_pose_out=userdata.humans_pose
        userdata.person_label_out=userdata.person_label
        
        return 'succeeded'
         
         
         
   
class face_detection(smach.State):
  def __init__(self):
      smach.State.__init__(self, 
                           outcomes=['succeeded', 'next','move',  'failed', 'preempted'],
                           input_keys = ['pose_list', 'id','humans_pose','sm_input','person_label'],
                           output_keys=['pose_list_output','id_out','face_list','humans_pose_out','bodies_list','person_label_out'])
      
      self.srv_name_face_detection='/cob_people_detection/detect_people'    
        
      self.object_pose=Pose()
      self.bbpose=Pose()
      
  def execute(self, userdata):
        

        if self.preempt_requested():
           self.service_preempt()
           return 'preempted'
         
        #searching for all people 
        if (userdata.person_label==''):
 
            try:
                rospy.wait_for_service(self.srv_name_face_detection,10)
            except rospy.ROSException, e:
                print "Service not available: %s"%e
                return 'next'
    
            # call object detection service
            try:
                detector_service = rospy.ServiceProxy(self.srv_name_face_detection, DetectPeople)
                req = DetectPeopleRequest()
                res = detector_service(req)
                if len(res.people_list.detections)==0:
                   userdata.id_out=userdata.id+1
                   userdata.pose_list_output=userdata.pose_list
                   return 'next'    
                 
                userdata.face_list=res.people_list
                userdata.id_out=userdata.id
                userdata.pose_list_output=userdata.pose_list
                userdata.humans_pose_out=userdata.humans_pose
                userdata.bodies_list=[]
                userdata.person_label_out=userdata.person_label
    
                return 'succeeded'
                
    
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    
                return 'failed'
            
        #searching for a special label    
        try:
            rospy.wait_for_service(self.srv_name_face_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            return 'next'

        # call object detection service
        try:
            detector_service = rospy.ServiceProxy(self.srv_name_face_detection, DetectPeople)
            req = DetectPeopleRequest()
            res = detector_service(req)
            if len(res.people_list.detections)==0:
               userdata.id_out=userdata.id+1
               userdata.pose_list_output=userdata.pose_list
               return 'move'     
            for i in range (0,len(res.people_list.detections)):
                if(res.people_list.detections[i].label==userdata.person_label):

                    userdata.face_list=res.people_list.detections[i]
                    userdata.id_out=userdata.id
                    userdata.pose_list_output=userdata.pose_list
                    userdata.humans_pose_out=userdata.humans_pose
                    userdata.bodies_list=[]
                    userdata.person_label_out=userdata.person_label

                    return 'succeeded'
            return 'move'

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            return 'failed'
class body_detection(smach.State):
  def __init__(self):
      smach.State.__init__(self, 
                           outcomes=['succeeded', 'retry',  'failed', 'preempted'],
                           input_keys = ['pose_list', 'id','humans_pose','person_label'],
                           output_keys=['pose_list_output','id_out','bodies_list','humans_pose_out','face_list','person_label_out'])
      
      self.srv_name_body_detection='/detect_bodies'    
        
      self.object_pose=Pose()
      self.bbpose=Pose()
      
  def execute(self, userdata):
        print userdata.id
        if self.preempt_requested():
           self.service_preempt()
           return 'preempted'
         
         
         
         
        try:
            rospy.wait_for_service(self.srv_name_body_detection,10)
        except rospy.ROSException, e:
            print "Service not available: %s"%e
            return 'failed'

        # call object detection service
        try:
            body_detector_service = rospy.ServiceProxy(self.srv_name_bod_detection, getBodyDetections)
            req = getBodyDetectionsRequest()
            res = body_detector_service(req)
            #if len(res.bodies_list)==0:


               #userdata.pose_list_output=userdata.pose_list
               #return 'retry'     
            userdata.bodies_list=res.bodies_list
            userdata.id_out=userdata.id
            userdata.pose_list_output=userdata.pose_list
            userdata.humans_pose_out=userdata.humans_pose
            
            userdata.face_list=PeopleDetectionArray()
            userdata.person_label_out=userdata.person_label

            return 'succeeded'
            

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            return 'failed'
                
        
class compare_detections(smach.State):
  def __init__(self):
      smach.State.__init__(self, 
                           outcomes=['succeeded', 'retry',  'failed', 'preempted'],
                           input_keys = ['pose_list','face_list','bodies_list', 'id','humans_pose','person_label'],
                           output_keys=['pose_list_out','id_out','humans_pose_out','person_label'])
      
      self.srv_name_compare='compare_hs_detections'
      self.human=[]
  def execute(self, userdata):
      
      print userdata.id
      if self.preempt_requested():
           self.service_preempt()
           return 'preempted'
             
      if (userdata.person_label==''):

          self.human_array=userdata.humans_pose 

             
          try:
                rospy.wait_for_service(self.srv_name_compare,10)
          except rospy.ROSException, e:
                print "Service not available: %s"%e
                return 'failed'
        
            # call object detection service
          try:
                detector_service = rospy.ServiceProxy(self.srv_name_compare, Comp_HS_Detections)
                comp=Comp_HS_DetectionsRequest()
                for i in range(len(userdata.face_list.detections)):
                    #print 'face'
                    pose=Pose()
                    pose.position.x=userdata.face_list.detections[i].pose.pose.position.x
                    pose.position.z=userdata.face_list.detections[i].pose.pose.position.z
                    pose.position.y=userdata.face_list.detections[i].pose.pose.position.y
                    pose.orientation.x=userdata.face_list.detections[i].pose.pose.orientation.x
                    pose.orientation.z=userdata.face_list.detections[i].pose.pose.orientation.z
                    pose.orientation.y=userdata.face_list.detections[i].pose.pose.orientation.y
                    
                    
                    comp.face_det=pose
                    comp.leg_det=userdata.pose_list[userdata.id]
                    res = detector_service(comp)
                    if res.human_detected==True:
                      self.human=detect_human()
                      self.human.is_person=0
                      self.human.label=userdata.face_list.detections[i].label
                      self.human.pose=userdata.pose_list[userdata.id]
                      self.human.probability=40
                      if(userdata.id<3):
                        self.human.probability=self.human.probability+20
                      self.human_array.detect_human.append(self.human)
                        
                for i in range(len(userdata.bodies_list)):
                    #print 'bodies'
                    pose=Pose()
                    pose.position.x=userdata.bodies_list[i].position.x
                    pose.position.z=userdata.bodies_list[i].position.z
                    pose.position.y=userdata.bodies_list[i].position.y
                    pose.orientation.x=userdata.bodies_list[i].orientation.x
                    pose.orientation.z=userdata.bodies_list[i].orientation.z
                    pose.orientation.y=userdata.bodies_list[i].orientation.y
                    
                    
                    comp.face_det=pose
                    comp.leg_det=userdata.pose_list[userdata.id]
                    res = detector_service(comp)
                    if res.human_detected==True:
                      self.human=detect_human()
                      self.human.is_person=0
                      self.human.label=userdata.face_list.detections[i].label
                      self.human.pose=userdata.pose_list[userdata.id]
                      self.human.probability=40
                      if(userdata.id<3):
                          self.human.probability=self.human.probability+20
                      self.human_array.detect_human.append(self.human)
                            
                    
                        
          
          except rospy.ServiceException, e:
                print "Service call failed: %s"%e
    
                return 'failed'
            
          userdata.humans_pose_out=self.human    
          print 'ID %d' % (userdata.id)
          print 'laenge  %d' % ( len(userdata.pose_list))
          if userdata.id+1>len(userdata.pose_list)-1:
              print self.human
              return 'succeeded'
          userdata.id_out=userdata.id+1
          userdata.pose_list_out=userdata.pose_list
          userdata.person_label_out=userdata.person_label
      
          return 'retry'
      
      
      
      #compare label
      
      
      self.human=detect_human()
      self.human.is_person=1
      self.human.label=userdata.person_label 
      self.human.pose=userdata.pose_list[userdata.id]
      self.human.probability=20
      if(userdata.id<3):
        self.human.probability=self.human.probability+20
      try:
            rospy.wait_for_service(self.srv_name_compare,10)
      except rospy.ROSException, e:
            print "Service not available: %s"%e
            return 'failed'
    
        # call object detection service
      try:
            compare_service = rospy.ServiceProxy(self.srv_name_compare, Comp_HS_Detections)
            comp=Comp_HS_DetectionsRequest()
           
            pose=Pose()
            pose.position.x=userdata.face_list.pose.pose.position.x
            pose.position.z=userdata.face_list.pose.pose.position.z
            pose.position.y=userdata.face_list.pose.pose.position.y
            pose.orientation.x=userdata.face_list.pose.pose.orientation.x
            pose.orientation.z=userdata.face_list.pose.pose.orientation.z
            pose.orientation.y=userdata.face_list.pose.pose.orientation.y
                
                
            comp.face_det=pose
            comp.leg_det=userdata.pose_list[userdata.id]
            res = compare_service(comp)
            if res.human_detected==True:
                self.human.probability=self.human.probability+20
                    
            
                            
                    
      
      except rospy.ServiceException, e:
            print "Service call failed: %s"%e

            return 'failed'
        
      userdata.humans_pose_out=self.human    
      print self.human    
      return 'succeeded'

      