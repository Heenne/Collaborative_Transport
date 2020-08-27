import smach
import FormationControlStates as st

class FormationControlStateMachine(smach.StateMachine): 
    def __init__(self,slave_namespaces):
        smach.StateMachine.__init__(self,   outcomes=['formation_control_done',"formation_control_error"])
       
        with self:      
            smach.StateMachine.add("Idle",st.FormationControlIdleState(),
                                    transitions={   'enable':'EnableControl',
                                                    'disable':'DisableControl',
                                                    'stop':'formation_control_done',
                                                    "error":"formation_control_error"})
                                                    
            smach.StateMachine.add('EnableControl',st.FormationControlServiceState(slave_namespaces,"slave_controller/enable_controller"),
                                    transitions={'called':'Idle'})

            smach.StateMachine.add('DisableControl',st.FormationControlServiceState(slave_namespaces,"slave_controller/disable_controller"),
                                    transitions={'called':'Idle'})