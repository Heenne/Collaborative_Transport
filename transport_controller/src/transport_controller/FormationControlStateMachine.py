import smach
import FormationControlStates as st
import MoveStates as mv
import States

class FormationControlStateMachine(smach.StateMachine): 
    def __init__(self,base_ns,arm_ns):
        smach.StateMachine.__init__(self,outcomes=['formation_control_move','formation_control_stop',"formation_control_error"])
       
        with self:         
            smach.StateMachine.add("Idle",st.FormationControlIdleState(),
                                    transitions={   'enable':'EnableControl',
                                                    'disable':'ReleaseObject',
                                                    'stop':'formation_control_stop',
                                                    "error":"formation_control_error",
                                                    "move":"formation_control_move"})
                                                    
            smach.StateMachine.add('EnableControl',
                                    st.FormationControlServiceState(base_ns,"slave_controller/enable_controller"),
                                    transitions={'called':'DriveToGrip'})
            
            
            smach.StateMachine.add("DriveToGrip",
                                    mv.DrivePoseState(arm_ns,"grip"),
                                    transitions={"done":"Adjust"})
            
            smach.StateMachine.add("Adjust",    States.WaitTriggerState(0.1,"adjusted"),
                                                                    transitions={"adjusted":"LinkObject"})


            smach.StateMachine.add("LinkObject", st.LinkObjectState(arm_ns),transitions={"linked":"Idle"})


            smach.StateMachine.add("ReleaseObject", st.ReleaseObjectState(arm_ns),transitions={"released":"DriveToMove"})

            smach.StateMachine.add("DriveToMove",
                                    mv.DrivePoseState(arm_ns,"drive"),
                                    transitions={"done":"DisableControl"})


            smach.StateMachine.add('DisableControl',
                                    st.FormationControlServiceState(base_ns,"slave_controller/disable_controller"),
                                    transitions={'called':'Idle'})            
           