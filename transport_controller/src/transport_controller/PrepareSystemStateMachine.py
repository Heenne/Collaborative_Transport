
import smach
import States as st

class PrepareSystemStateMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,   outcomes=['preparation_done'],
                                            input_keys=["slaves_ns"],
                                            output_keys=["slaves"])
        with self:      

            smach.StateMachine.add('Start',st.StartState(),
                            transitions={'initialized':'Initialized'})

            smach.StateMachine.add('Initialized',st.InitializedState(),
                                    transitions={'start':'CalcPoses','stay':'Initialized'})

            smach.StateMachine.add('CalcPoses', st.CalcPosesState(),
                                    transitions={'calculation_done':'preparation_done',
                                                 'calculation_error':'Initialized'})