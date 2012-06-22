#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_human_sensing')
import rospy
import smach
import smach_ros

from human_sensing_states import *
from srs_human_sensing.msg import *

def main():

    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'],
                                input_keys=['sm_input'],
                                output_keys=['humans_pose'])

    test=human_sensing_sm_input()
    test.label='alex'
    p=Pose2D()
    p.x=5
    p.y=5
    p.theta=5
    test.pose_furniture.append(p)
    sm.userdata.sm_input=test

    with sm:
               
            smach.StateMachine.add('Leg Detection', leg_detection(),
                    transitions={'succeeded':'Move to better position', 'failed':'failed', 'preempted':'preempted','retry':'Leg Detection'})
            
            smach.StateMachine.add('Move to better position', move_to_better_position(),
                    transitions={'succeeded':'Face detection','failed':'failed', 'preempted':'preempted'},
                    remapping={'pose_list_out':'pose_list','humans_pose_out':'humans_pose',
                               'id_out':'id','person_label_out':'person_label'})
            smach.StateMachine.add('Face detection', face_detection(),
                    transitions={'succeeded':'Body detection','retry':'Move to better position', 'failed':'failed','preempted':'preempted'},
                    remapping={'pose_list_output':'pose_list','humans_pose_out':'humans_pose',
                               'id_out':'id'})
            smach.StateMachine.add('Body detection', body_detection(),
                    transitions={'succeeded':'compare_detections', 'failed':'failed','preempted':'preempted'},
                    remapping={'pose_list_output':'pose_list','humans_pose_out':'humans_pose',
                               'id_out':'id','face_list_out':'face_list'})
            smach.StateMachine.add('compare_detections', compare_detections(),
                    transitions={'succeeded':'succeeded','retry':'Move to better position', 'failed':'failed','preempted':'preempted'},
                    remapping={'pose_list_output':'pose_list','humans_pose_out':'humans_pose',
                               'id_out':'id'})

    outcome = sm.execute()


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
    
if __name__ == '__main__':
    main()
