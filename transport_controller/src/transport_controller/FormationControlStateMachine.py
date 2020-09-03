import smach
import FormationControlStates as st
import MoveStates as mv

class FormationControlStateMachine(smach.StateMachine): 
    def __init__(self,slave_namespaces):
        smach.StateMachine.__init__(self,outcomes=['formation_control_done',"formation_control_error"])
       
        with self:         
            smach.StateMachine.add("Idle",st.FormationControlIdleState(),
                                    transitions={   'enable':'EnableControl',
                                                    'disable':'DisableControl',
                                                    'stop':'formation_control_done',
                                                    "error":"formation_control_error"})
                                                    
            smach.StateMachine.add('EnableControl',st.FormationControlServiceState(slave_namespaces,"slave_controller/enable_controller"),
                                    transitions={'called':'DriveToGrip'})
            
            smach.StateMachine.add("DriveToGrip",mv.DrivePoseState(["mur/ur","miranda/panda"],"grip"),
                                                            transitions={"done":"Idle"})

            smach.StateMachine.add('DisableControl',st.FormationControlServiceState(slave_namespaces,"slave_controller/disable_controller"),
                                    transitions={'called':'DriveToMove'})

            smach.StateMachine.add("DriveToMove",mv.DrivePoseState(["mur/ur","miranda/panda"],"drive"),
                                                                    transitions={"done":"Idle"})