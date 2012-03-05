# ROS imports
import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
import threading
import tf
from std_msgs.msg import String, Bool, Int32
from geometry_msgs.msg import *

"""
current_task_info is a global shared memory for exchanging task information among statuses 

smach is slow on passing large amount of userdata. Hence they are stored under goal_structure as global variable

srs_dm_action perform one task at a time and maintain a unique session id.  
"""

listener = tf.TransformListener()

class goal_structure():   
    
    def __init__(self):
        
        #goal of the high level task
        self.task_name =""
        
        #task parameter
        self.task_parameter=""
        
        #Information about last step, use Last_step_info_msg 
        self.last_step_info = list()
        
        #customised pre-empty signal received or not
        self.preemptied = False
        
        #reference to the action server
        self._srs_as=""
        
        ## backward compatible need to be revised after the integration meeting         
        #feedback publisher, intervention required
        self.pub_fb = rospy.Publisher('fb_executing_solution', Bool)
        #feedback publisher, operational state
        self.pub_fb2 = rospy.Publisher('fb_executing_state', String)
        ## backward compatible need to be revised after the integration meeting  
        
        self.object_in_hand = False
        
        self.object_identified = False
        
        #True already adjusted, False not adjusted yet
        self.post_grasp_adjusted = False
        
        
        # this can be replaced by the tray service with real robot
        self.object_on_tray = False
        
        self.arm_folded_ready_for_transfer = False    #arm in hold or folded position
        
        self.lock = threading.Lock()
        
        self.customised_preempt_required = False
        
        self.customised_preempt_acknowledged = False
        
        self.pause_required = False
        
        self.stop_required = False 
        
        self.stop_acknowledged = False
        
    def get_post_grasp_adjustment_state(self):
        self.lock.acquire()
        #True already adjusted, False not adjusted yet
        value = self.post_grasp_adjusted
        self.lock.release()
        return value 
    
    def set_post_grasp_adjustment_state(self, value):
        self.lock.acquire()
        #True already adjusted, False not adjusted yet
        self.post_grasp_adjusted = value
        self.lock.release()

    def get_object_identification_state(self):
        self.lock.acquire()
        value = self.object_identified
        self.lock.release()
        return value 
    
    def set_object_identification_state(self, value):
        self.lock.acquire()
        self.object_identified = value
        self.lock.release()
    

    def get_pause_required(self):
        self.lock.acquire()
        value = self.pause_required
        self.lock.release()
        return value 
    
    def get_customised_preempt_required(self):
        self.lock.acquire()
        value = self.customised_preempt_required
        self.lock.release()
        return value 
    
    def get_customised_preempt_acknowledged(self):
        self.lock.acquire()
        value = self.customised_preempt_acknowledged
        self.lock.release()
        return value 
        
    def get_stop_required(self):
        self.lock.acquire()
        value = self.stop_required
        self.lock.release()
        return value 

    def get_stop_acknowledged(self):
        self.lock.acquire()
        value = self.acknowledged
        self.lock.release()
        return value 
    
    def set_pause_required(self,value):
        self.lock.acquire()
        self.pause_required = value
        self.lock.release()

    def set_customised_preempt_required(self,value):
        self.lock.acquire()
        self.customised_preempt_required = value
        self.lock.release()

    
    def set_customised_preempt_acknowledged(self, value):
        self.lock.acquire()
        self.customised_preempt_acknowledged = value
        self.lock.release()

        
    def set_stop_required(self, value):
        self.lock.acquire()
        self.stop_required = value
        self.lock.release()

    def set_stop_acknowledged(self, value):
        self.lock.acquire()
        self.acknowledged = value
        self.lock.release()
    
    #checking if the current atomic operation can be stopped or not
    #true can be stopped in the middle
    def stopable(self):
        #List of conditions which robot should not be stopped in the middle
        self.lock.acquire()
        #condition 1:
        #If a object has been grasped, operation should not be stopped until the object is released or arm is folded ready for transfer
        if self.object_in_hand and not self.arm_folded_ready_for_transfer:
            outcome = False
        else:
            outcome = True
        self.lock.release()
        
        return outcome

    
    def reset(self):
        
        self.__init__()
        gc.collect()
        
    def get_robot_pos(self):
        try:
            (trans,rot) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return None
            
        rb_pose = Pose2D()
        rb_pose.x = trans[0]
        rb_pose.y = trans[1]
        rb_pose_rpy = tf.transformations.euler_from_quaternion(rot)
        rb_pose.theta = rb_pose_rpy[2]
        
        return rb_pose

        

current_task_info = goal_structure() 
