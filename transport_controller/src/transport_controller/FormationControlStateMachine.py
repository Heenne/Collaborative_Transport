import smach
import FormationControlStates as st

class FormationControlStateMachine(smach.StateMachine): 
    def __init__(self,slave_namespaces):
        smach.StateMachine.__init__(self,   outcomes=['formation_control_done'])
       
        with self:      
            smach.StateMachine.add("Idle",st.FormationControlIdleState(),
                                    transitions={   'enable':EnableControl},
                                                    'disable':'DisableControl',
                                                    'stop':'formation_control_done'})
                                                    
            smach.StateMachine.add('EnableControl',st.FormationControlServiceState(slave_namespaces,"enable_controller"),
                                    transitions={'called':'DisableControl'})

            smach.StateMachine.add('DisableControl',st.FormationControlServiceState(slave_namespaces,"enable_controller"),
                                    transitions={'called':'CalcPoses'})