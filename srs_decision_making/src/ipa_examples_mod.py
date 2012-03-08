# ROS imports
import roslib
roslib.load_manifest('srs_decision_making')
#import copy
import rospy
import smach
#import smach_ros

from std_msgs.msg import String, Bool, Int32
from cob_srvs.srv import Trigger
from math import *
import time
from kinematics_msgs.srv import *

#import actionlib
# ROS imports
import roslib
roslib.load_manifest('srs_states')
import rospy
import smach
# include script server, to move the robot
from simple_script_server import *
sss = simple_script_server()

# msg imports
from geometry_msgs.msg import *
from shared_state_information import *

#from gazebo.srv import *
#import gazebo.msg as gazebo







            

"""
Below dummy generic states are copied and modified based on IPA examples for testing purpose
They should be replaced by real states from other SRS components in the future  

Basic states related to robot includes:

approach_pose()
approach_pose_without_retry()
select_grasp()
grasp_side()
grasp_top()
open_door()
put_object_on_tray()
detect_object()

Only dummy outputs are given for testing purpose
"""





class select_post_table_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed','preempted'], input_keys=['post_table_pos'], output_keys=['post_table_pos'])
        #self.counter = 0
    
    def execute(self, userdata):
        global current_task_info
        
        if current_task_info.get_post_grasp_adjustment_state()==True:  # already adjusted
            return 'succeeded'            
        else:
            pos=current_task_info.get_robot_pos()
            
            if pos ==None:
                userdata.post_table_pos=''
                return 'failed'
            else:
                current_task_info.set_post_grasp_adjustment_state(True)  
                pos.x = pos.x + 0.15 * cos(pos.theta)
                pos.y = pos.y + 0.15 * sin(pos.theta)
                userdata.post_table_pos = list()
                userdata.post_table_pos.append(pos.x)
                userdata.post_table_pos.append(pos.y)
                userdata.post_table_pos.append(pos.theta)
                return 'succeeded'
        

class select_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['got_to_next_pose', 'no_more_pose', 'failed','preempted'], 
                             input_keys=['target_base_pose_list'], output_keys=['current_target_base_pose'])
    def execute(self, userdata):
        userdata.current_target_base_pose='home'
        return 'got_to_next_pose'


## Put object on tray side state
#
# This state puts a side grasped object on the tray
class put_object_on_tray(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed' ,'preempted'],
            input_keys=['grasp_categorisation'])
        
        
    def execute(self, userdata):
        #TODO select position on tray depending on how many objects are on the tray already
        global current_task_info
        
        if current_task_info.object_in_hand and not current_task_info.object_on_tray:
        
            # move object to frontside
            handle_arm = sss.move("arm","grasp-to-tray",False)
            sss.sleep(2)
            sss.move("tray","up")
            handle_arm.wait()
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            
            # release object
            if userdata.grasp_categorisation == 'side':
                sss.move("sdh","cylopen")
            elif userdata.grasp_categorisation == 'top':
                sss.move("sdh","spheropen")
                
            #object is transfered from hand to tray 
            current_task_info.object_in_hand = False
            current_task_info.object_on_tray = True
            
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # move arm to backside again
        handle_arm = sss.move("arm","tray-to-folded",False)
        sss.sleep(3)
        sss.move("sdh","home")
        handle_arm.wait()
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        else:
            return 'succeeded'


#verify_object FROM PRO+IPA, the interface still need to be clarified 
class verify_object(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_verified','no_object_verified','failed','preempted'],
                                input_keys=['reference_to_map','object_name_list'],
                                output_keys=['updated_object_list'])
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        Extract objects from current point map
        """
        updated_object = ""   #updated pose information about the object
        return 'failed'  

#scan environment from IPA, the interface still need to be clarified    
class update_env_model(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'not_completed', 'failed', 'preempted'])
        
    def execute(self,userdata):
        # user specify key region on interface device for detection
        """
        Get current point map
        """
        #map_reference = ""   
        return 'succeeded'      


#verify_object FROM PRO+IPA, the interface still need to be clarified 
#for integration meeting
class object_verification_simple(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['object_verified','no_object_verified','failed','preempted'],
                                input_keys=['target_object_name','target_object_hh_id','target_object_pose'],
                                output_keys=['target_object_pose'])
        #object verified: the expected object has been found, the revised pose is in the output_put key verified_target_object_pose
        #no_object_verified: no object found, given up
        
                
    def execute(self,userdata):
        # user specify key region on interface device for detection
        
        
        
        #dummy code for testing
        #verified object is in the exact expected position
        #return object_verfied
        
        userdata.target_object_pose = userdata.target_object_pose
        return 'object_verified'  

    
"""
## Detect state
#
# This state will try to detect an object.
class detect_object(smach.State):
    def __init__(self,object_name = "",max_retries = 1):
        smach.State.__init__(
            self,
            outcomes=['succeeded','retry','no_more_retries','failed'],
            input_keys=['object_name'],
            output_keys=['object'])

        self.object_name = object_name
        

    def execute(self, userdata):
        # determine object name
        if self.object_name != "":
            object_name = self.object_name
        elif type(userdata.object_name) is str:
            object_name = userdata.object_name
        else: # this should never happen
            rospy.logerr("Invalid userdata 'object_name'")
            self.retries = 0
            return 'failed'

        return 'succeeded'





class move_head(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['torso_pose'])
        
        self.torso_poses = []
        self.torso_poses.append("home")
        self.torso_poses.append("left")
        self.torso_poses.append("right")


    def execute(self, userdata):
        sss.move("torso",userdata.torso_pose)
        return 'succeeded'




## Deliver object state
#
# This state will deliver an object which should be on the tray.
class deliver_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
            outcomes=['succeeded', 'retry', 'failed'])

    def execute(self, userdata):
        
        #sss.say(["Here is your " + userdata.object_name + ". Please help yourself."],False)
        sss.move("torso","nod",False)
        
        try:
            rospy.wait_for_service('/tray/check_occupied',10)
        except rospy.ROSException, e:
            rospy.loginfo("\n\nService not available: %s", e)
	    rospy.loginfo('\n\nIf the tray has been emptied? Please enter Yes/No - Y/N')
	    inp = raw_input()
	    if inp == 'y' or inp == 'Y':
	        
		sss.move("tray","down",False)
                sss.move("torso","nod",False)
		return  'succeeded'
	    else:
		return 'failed'

        time = rospy.Time.now().secs
        loop_rate = rospy.Rate(5) #hz
        while True:
            if rospy.Time.now().secs-time > 20:
                return 'retry'
            try:
                tray_service = rospy.ServiceProxy('/tray/check_occupied', CheckOccupied)            
                req = CheckOccupiedRequest()
                res = tray_service(req)
                print "waiting for tray to be not occupied any more"
                if(res.occupied.data == False):
                    break
            except rospy.ServiceException, e:
                print "Service call failed: %s", e
                return 'failed'
            sss.sleep(2)
        
        sss.move("tray","down",False)
        sss.move("torso","nod",False)
        
        return 'succeeded'
"""

