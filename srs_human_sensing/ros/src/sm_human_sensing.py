

class sm_human_sensing(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,
                                    outcomes=['succeeded', 'failed', 'preempted'],
                                    input_keys=[],
                                    output_keys=['pose_leg','pose_face'])

       

        with self:
                      
            smach.StateMachine.add('Leg Detection', leg_detection(),
                    transitions={'succeeded':'Move to better position', 'failed':'failed', 'preempted':'preempted'})
            
            smach.StateMachine.add('Move to better position', move_to_better_position(),
                    transitions={'succeeded':'Face detection','failed':'failed', 'preempted':'preempted'})
            
            smach.StateMachine.add('Face detection', face_detection(),
                    transitions={'succeeded':'succeeded','retry':'Move to better position', 'failed':'failed','preempted':'preempted'})

