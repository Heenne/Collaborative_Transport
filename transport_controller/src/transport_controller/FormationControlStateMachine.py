import smach
import States as st

class PrepareSystemStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,   outcomes=['formation_control_done'],
                                            input_keys=["slaves_ns"],
                                            output_keys=["slaves"])
        with self:      

            smach.StateMachine.add( 'EnableController',st.EnableControllerState(),
                                    transitions={'enabled':'FormationControl'})
            smach.StateMachine.add( 'FormationControl',st.FormationControlState(),
                                    transitions={'wait':'FormationControl',
                                                 'disable':'EnableController'}
                                                 'stop':'formation_control_done')
