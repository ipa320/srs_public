# ROS imports
import roslib
roslib.load_manifest('srs_states')
import rospy
import smach

from simple_script_server import *
sss = simple_script_server()

from shared_state_information import *


## Approach pose state (without retry)
#
# This state tries once to move the robot to the given pose.
# 
# Modified to handle user intervention and pre-empty request
##
class approach_pose_without_retry(smach.State):

    def __init__(self, pose = ""):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_completed','failed', 'preempted'],
            input_keys=['base_pose'])

        self.pose = pose
        self.counter =0
        #self.mode = "linear"
        self.mode = "omni"

    def execute(self, userdata):
        
        global current_task_info
        # robot moved, any previous detection is not valid anymore
        current_task_info.set_object_identification_state(False) 
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # determine target position
        if self.pose != "":
            pose = self.pose
        elif type(userdata.base_pose) is str:
            pose = userdata.base_pose
        elif type(userdata.base_pose) is list:
            pose = []
            pose.append(userdata.base_pose[0])
            pose.append(userdata.base_pose[1])
            pose.append(userdata.base_pose[2])
        else: # this should never happen
            rospy.logerr("Invalid userdata 'pose'")
            return 'failed'

        # try reaching pose
        handle_base = sss.move("base", pose, False, self.mode)
        move_second = False
        


        timeout = 0
        while not self.preempt_requested():
            try:
                #print "base_state = ", handle_base.get_state()
                if (handle_base.get_state() == 3) and (not move_second):
                    # do a second movement to place the robot more exactly
                    handle_base = sss.move("base", pose, False, self.mode)
                    move_second = True
                elif (handle_base.get_state() == 3) and (move_second):
                    return 'succeeded'        
                elif (handle_base.get_state() == 2 or handle_base.get_state() == 4):  #error or paused
                    rospy.logerr("base not arrived on target yet")
                    return 'not_completed'
            except rospy.ROSException, e:
                error_message = "%s"%e
                rospy.logerr("unable to check hdl_base state, error: %s", error_message)
                rospy.sleep(0.5)

            # check if service is available
            service_full_name = '/base_controller/is_moving'
            try:
                #rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
                rospy.wait_for_service(service_full_name,3)
            except rospy.ROSException, e:
                error_message = "%s"%e
                rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
                return 'failed'
        
            # check if service is callable
            try:
                is_moving = rospy.ServiceProxy(service_full_name,Trigger)
                resp = is_moving()
            except rospy.ServiceException, e:
                error_message = "%s"%e
                rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                return 'failed'
        
            # evaluate service response
            if not resp.success.data: # robot stands still
                if timeout > 10:
                    sss.say(["I can not reach my target position because my path or target is blocked, I will abort."],False)
        
                    try:
                        rospy.wait_for_service('base_controller/stop',10)
                        stop = rospy.ServiceProxy('base_controller/stop',Trigger)
                        resp = stop()
                    except rospy.ServiceException, e:
                        error_message = "%s"%e
                        rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
                    except rospy.ROSException, e:
                        error_message = "%s"%e
                        rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)        
                    return 'not_completed'
                else:
                    timeout = timeout + 1
                    rospy.sleep(1)
            else:
                timeout = 0
        if self.preempt_requested():
            self.service_preempt()
            #handle_base.set_failed(4)
            handle_base.client.cancel_goal()
            return 'preempted'
        return 'failed'
