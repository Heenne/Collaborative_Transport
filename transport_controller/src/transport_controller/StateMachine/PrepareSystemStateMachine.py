import smach
import PrepareSystemStates as st
import States

class PrepareSystemStateMachine(smach.StateMachine):
    def __init__(self,base_namespaces,arm_namespaces):
        smach.StateMachine.__init__(self,   outcomes=['preparation_done','preparation_error'],
                                            output_keys=["slaves"])
        with self:      

            smach.StateMachine.add('Start',st.StartState(base_namespaces,arm_namespaces),
                                    transitions={   'startup_done':'Idle',
                                                    "references_error":"preparation_error"})

            smach.StateMachine.add("Idle",    States.WaitTriggerState(0.1,"start"),
                                    transitions={"start":"preparation_done"})

           