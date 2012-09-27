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
from cob_kinematics.srv import *
from arm_navigation_msgs.srv import *

TIP_LINK='sdh_palm_link'

def parse_cartesian_param(param, now = None):
    if now is None:
        now = rospy.Time.now()
    ps = PoseStamped()
    ps.pose.orientation.w = 1.0
    ps.header.stamp = now
    if type(param) is not PoseStamped and param is not None:
        ps.header.frame_id = param[0]
        if len(param) > 1:
            ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
        if len(param) > 2:
            ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
    else:
        ps = param
    return ps
    
def parse_cartesian_parameters(arm_name, parameters):
    now = rospy.Time.now()
    
    # parse pose_target
    param = parameters
    second_param = None
    if type(parameters) is list and len(parameters) > 0:
        if type(parameters[0]) is not str:
            param = parameters[0]
            if len(parameters) > 1:
                second_param = parameters[1]

    pose_target = parse_cartesian_param(param, now)

    # parse pose_origin
    param = second_param
    ps = PoseStamped()
    ps.pose.orientation.w = 1.0
    ps.header.stamp = pose_target.header.stamp
    ps.header.frame_id = rospy.get_param("/cob_arm_kinematics/"+arm_name+"/tip_name")
    if type(param) is not PoseStamped:
         if param is not None and len(param) >=1:
            ps.header.frame_id = param[0]
            if len(param) > 1:
                ps.pose.position.x,ps.pose.position.y,ps.pose.position.z = param[1]
            if len(param) > 2:
                ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w = quaternion_from_euler(*param[2])
    else:
        ps = param
    return pose_target,ps
    
def get_joint_goal(arm_name, target):
    SetPlanningSceneDiffService = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
    planning_scene_request = SetPlanningSceneDiffRequest()
    planning_scene_response = SetPlanningSceneDiffService(planning_scene_request)
    ps_target, ps_origin = parse_cartesian_parameters(arm_name, target)
    iks = rospy.ServiceProxy("/cob_ik_wrapper/arm/get_ik_extended", GetPositionIKExtended)
    req = GetPositionIKExtendedRequest()
    req.ik_pose = ps_origin.pose
    req.constraint_aware = True
    req.timeout = rospy.Duration(5.0)
    req.ik_request.ik_link_name = ps_origin.header.frame_id
    req.ik_request.pose_stamped = ps_target
    req.ik_request.ik_seed_state.joint_state.name = ["arm_%d_joint" % (d+1) for d in range(7)]
    req.ik_request.ik_seed_state.joint_state.position = [0.0]*7
    res = iks(req)
    return res.solution.joint_state, res.error_code

# base class for all move arm states
class _move_arm_base(smach.State):
    def __init__(self, additional_input_keys = []):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_completed','failed', 'preempted'],
            output_keys=['pose_id'],
            input_keys=['poses'] + additional_input_keys)
        
    def moveArm(self,userdata,poses,mode):
        ret  = 'not_completed'
        userdata.pose_id = None
        
        for pose_id in range(len(poses)): # for each pose
        
            if self.preempt_requested(): # return if preempt requested
                return 'preempted'

            for i in range(5):
		js, err = get_joint_goal('arm',[poses[pose_id],[TIP_LINK]])
		if err.val == err.SUCCESS:
		    break

	    if err.val != err.SUCCESS:
		print err.val
		continue            

            # start movement
            handle_arm= sss.move('arm',[list(js.position)],False, mode=mode)

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
        _move_arm_base.__init__(self,['mode'])
    def execute(self, userdata):
        return self.moveArm(userdata,userdata.poses,userdata.mode)
        
# move arm state with planning
class move_arm_planned(_move_arm_base):
    def __init__(self):
        _move_arm_base.__init__(self)
    def execute(self, userdata):
        return self.moveArm(userdata, userdata.poses,'planned')
# move arm stat without planning
class move_arm_unplanned(_move_arm_base):
    def __init__(self):
        _move_arm_base.__init__(self)
    def execute(self, userdata):
        return self.moveArm(userdata, userdata.poses,'unplanned')
