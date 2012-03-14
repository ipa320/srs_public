# ROS imports
import roslib
roslib.load_manifest('srs_states')
import rospy
import smach

from simple_script_server import *
sss = simple_script_server()

import tf
from tf.transformations import *
from kinematics_msgs.srv import *

from shared_state_information import *

# base class for all move arm states
class _move_arm_base(smach.State):
   def __init__(self, additional_input_keys = []):
       smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_completed','failed', 'preempted'],
            output_keys=['pose_id'],
            input_keys=['poses'] + additional_input_keys)

    def poseStampedtoSSS(self,pose_stamped):
        pose = pose_stamped.pose
        euler = euler_from_quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w)
        return [[pose_stamped.frame_id, [pose.position.x,pose.position.y,pose.position.z],list(euler)]]

        
    def moveArm(self,userdata,poses,planned):
        ret  = 'not_completed'
        userdata.pose_id = None
        
        for pose_id in range(len(poses)): # for each pose
        
            if self.preempt_requested(): # return if preempt requested
                return 'preempted'

            ik_pose, error_pose = sss.calculate_ik(self.poseStampedtoSSS(poses[pose_id]))
            
            if error_pose is error_pose.NO_IK_SOLUTION:
                continue # if no solution was found, check next pose
            elif error_pose is error_pose.SUCCESS: # got solution
                # start movement
                if planned:
                    handle_arm= sss.move_planned('arm',[ik_pose],False)
                else:        
                    handle_arm= sss.move('arm',[ik_pose],False)
                
                # wait while movement
                r = rospy.Rate(10)
                preempted = False
                arm_state = -1
                while True:
                        preempted = self.preempt_requested()
                        arm_state = handle_arm.get_state()
                        if preempted or ( arm_state == 3) or (arm_state == 4):
                            break # stop waiting  
                        r.sleep()
                        
                # stop arm in any case
                sss.stop("arm")
                   
                if preempted:
                    ret = 'preempted'
                elif arm_state == 3:
                    ret = 'success'
                    userdata.pose_id = pose_id # keep pose_id for subsequent state
                else:
                    ret = 'failed'
                
                break
            else:
                ret = 'failed'
         return ret


# move arm state with flag for planning
class move_arm(_move_arm_base):
   def __init__(self):
       _move_arm_base.__init__(self,['planned'])
    def execute(self, userdata):
        return self.moveArm(userdata,userdata.poses,userdata.planned)
        
# move arm state with planning
class move_arm_planned(_move_arm_base):
   def __init__(self):
       _move_arm_base.__init__(self)
   def execute(self, userdata):
       return self.moveArm(userdata, userdata.poses,True)
# move arm stat without planning
class move_arm_unplanned(_move_arm_base):
   def __init__(self):
       _move_arm_base.__init__(self)
   def execute(self, userdata):
       return self.moveArm(userdata, userdata.poses,False)
