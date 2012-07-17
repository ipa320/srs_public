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

def callIKSolver(goal_pose):
    while(len(sss.arm_joint_positions) == 0): # dirty hack
        rospy.sleep(0.1)
    for i in range(5):
        iks = rospy.ServiceProxy('/srs_arm_kinematics/get_ik', GetPositionIK)

        req = GetPositionIKRequest();
        req.ik_request.ik_link_name = rospy.get_param("/srs_arm_kinematics/arm/tip_name");
        req.ik_request.ik_seed_state.joint_state.position = sss.arm_joint_positions
        req.ik_request.pose_stamped = goal_pose;
        req.timeout = rospy.Duration(5.0)
        resp = iks(req);
        if resp.error_code.val is resp.error_code.SUCCESS:
            break
    return (list(resp.solution.joint_state.position), resp.error_code);


# base class for all move arm states
class _move_arm_base(smach.State):
    def __init__(self, additional_input_keys = []):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_completed','failed', 'preempted'],
            output_keys=['pose_id'],
            input_keys=['poses'] + additional_input_keys)
        
    def moveArm(self,userdata,poses,planned):
        ret  = 'not_completed'
        userdata.pose_id = None
        
        for pose_id in range(len(poses)): # for each pose
        
            if self.preempt_requested(): # return if preempt requested
                return 'preempted'

            # start movement
            handle_arm = None
            if planned:
                handle_arm= sss.move_cartesian_planned('arm',[poses[pose_id],rospy.get_param("/srs_arm_kinematics/arm/tip_name")],False)
            else:        
                ik_pose, error_pose = callIKSolver(poses[pose_id])
                
                if error_pose.val is error_pose.NO_IK_SOLUTION:
                    continue # if no solution was found, check next pose
                elif error_pose.val is error_pose.SUCCESS: # got solution
                    print "move to",[ik_pose]
                    handle_arm= sss.move('arm',[ik_pose])#,False)
            if handle_arm is not None:
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
                    handle_arm.cancel()
                    ret = 'preempted'
                elif arm_state == 3:
                    ret = 'succeeded'
                    userdata.pose_id = pose_id # keep pose_id for subsequent state
                else:
                    ret = 'not_completed'
                
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
