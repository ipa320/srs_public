# ROS imports
import roslib
roslib.load_manifest('srs_states')
import rospy
import smach

from simple_script_server import *
sss = simple_script_server()

from shared_state_information import *
import random
from nav_msgs.msg import Odometry

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
        self.timeout = 30
        self.is_moving = False
        self.warnings = ["I can not reach my target position because my path or target is blocked.","My path is blocked.", "I can not reach my target position."]
        # Subscriber to base_odometry
        rospy.Subscriber("/base_controller/odometry", Odometry, self.callback)
        
        self.mode = "omni" 
        try:
            self.mode = rospy.get_param("srs/common/default_navigation_mode")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default value for navigation") 
        

    #Callback for the /base_controller/odometry subscriber
    def callback(self,msg):
        r = 0.01 # error range in m/s or rad/s
        if (abs(msg.twist.twist.linear.x) > r) or (abs(msg.twist.twist.linear.y) > r) or (abs(msg.twist.twist.angular.z) > r):
            self.is_moving = True
        else:
            self.is_moving = False
        return




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
        handle_base = sss.move("base", pose, mode=self.mode, blocking=False)
        #move_second = False
        
        # init variables
        stopping_time = 0.0
        announce_time = 0.0
        freq = 2.0 # Hz
        yellow = False

        
        # check for goal status
        while not rospy.is_shutdown():
            # stopped or paused due to intervention
            if self.preempt_requested():
                self.service_preempt()
                #handle_base.set_failed(4)
                handle_base.client.cancel_goal()
                sss.stop("base")
                return 'preempted'
            
            # finished with succeeded
            if (handle_base.get_state() == 3):
                sss.set_light('green')
                return 'succeeded'
            # finished with aborted
            elif (handle_base.get_state() == 4):
                sss.set_light('green')
                return 'not_completed'
            # finished with preempted or canceled
            elif (handle_base.get_state() == 2) or (handle_base.get_state() == 8):
                sss.set_light('green')
                return 'not_completed'
            # return with error
            elif (handle_base.get_error_code() > 0):
                print "error_code = " + str(handle_base.get_error_code())
                sss.set_light('red')
                return 'failed'

            # check if the base is moving
            loop_rate = rospy.Rate(freq) # hz
            if not self.is_moving: # robot stands still
                # increase timers
                stopping_time += 1.0/freq
                announce_time += 1.0/freq

                # abort after timeout is reached
                if stopping_time >= self.timeout:
                    sss.stop("base")
                    sss.set_light('green')
                    return 'not_completed'

                # announce warning after every 10 sec
                if announce_time >= 10.0:
                    sss.say([self.warnings[random.randint(0,len(self.warnings)-1)]],False)
                    announce_time = 0.0

                # set light to "thinking" after not moving for 2 sec
                if round(stopping_time) >= 2.0:
                    sss.set_light("blue")
                    yellow = False
                else:
                    # robot is moving
                    if not yellow:
                        sss.set_light("yellow")
                        yellow = True

                # sleep
                loop_rate.sleep()