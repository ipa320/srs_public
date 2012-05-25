#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_human_sensing')
import rospy
import smach
import smach_ros

from human_sensing_states import *

def main():
    
    rospy.init_node('smach_example_state_machine')
    sm = smach.StateMachine(outcomes=['succeeded', 'failed', 'preempted'])
                             #   input_keys=[],
                             #   output_keys=['pose_leg','pose_face'])

    

    with sm:
               
            smach.StateMachine.add('Leg Detection', leg_detection(),
                    transitions={'succeeded':'Move to better position', 'failed':'failed', 'preempted':'preempted','retry':'Leg Detection'})
            
            smach.StateMachine.add('Move to better position', move_to_better_position(),
                    transitions={'succeeded':'Face detection','failed':'failed', 'preempted':'preempted'})
            
            smach.StateMachine.add('Face detection', face_detection(),
                    transitions={'succeeded':'compare_detections','retry':'Move to better position', 'failed':'failed','preempted':'preempted'},
                    remapping={'pose_list_output':'pose_list',
                               'id_out':'id'})
            smach.StateMachine.add('compare_detections', compare_detections(),
                    transitions={'succeeded':'succeeded','retry':'Move to better position', 'failed':'failed','preempted':'preempted'},
                    remapping={'pose_list_output':'pose_list',
                               'id_out':'id'})

    outcome = sm.execute()


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
    
if __name__ == '__main__':
    main()