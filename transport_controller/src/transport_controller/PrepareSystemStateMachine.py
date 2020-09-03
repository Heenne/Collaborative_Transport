import smach
import PrepareSystemStates as st

class PrepareSystemStateMachine(smach.StateMachine):
    def __init__(self,slave_namespaces):
        smach.StateMachine.__init__(self,   outcomes=['preparation_done','preparation_error'],
                                            output_keys=["slaves"])
        with self:      

            smach.StateMachine.add('Start',st.StartState(slave_namespaces),
                                    transitions={   'startup_done':'Idle',
                                                    "references_error":"preparation_error"})

            smach.StateMachine.add('Idle',st.IdleState(),
                                    transitions={   'start':'preparation_done'})

           