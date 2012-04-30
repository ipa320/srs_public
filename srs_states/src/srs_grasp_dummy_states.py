#!/usr/bin/python

# dummy functions, 
# for maintaining package stability if srs_grasp is not working or compiled 
 
import roslib; roslib.load_manifest('srs_states')
import rospy
import smach

class select_srs_grasp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_possible', 'failed', 'preempted'],
            input_keys=['object', 'target_object_id'],
            output_keys=['grasp_configs', 'poses'])
        
        #default grasp categorisation
        #self.grasp_configuration = ""
        #self.object_id=9

    def execute(self, userdata):
        userdata.grasp_configs=''
        userdata.poses=''
        return 'failed'



class srs_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','not_completed','failed', 'preempted'], 
                             input_keys=['grasp_configuration','grasp_configuration_id'], 
                             output_keys=['grasp_categorisation'])
	
    def get_joint_state(self, msg):
        global current_joint_configuration
        current_joint_configuration = list(msg.desired.positions)
        rospy.spin()

    def execute(self, userdata):
        userdata.grasp_categorisation=''
        return 'failed'

# estimate the best grasp position
class grasp_base_pose_estimation(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['retry', 'not_retry', 'failed', 'preempted'],
            input_keys=['object','target_workspace_name'],
            output_keys=['base_pose'])
        
        self.counter = 0 


    def execute(self, userdata):
        userdata.base_pose=''
        return 'failed'

