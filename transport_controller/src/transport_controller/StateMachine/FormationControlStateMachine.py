import smach
import FormationControlStates as st
import MoveStates as mv
import States
import rospy

class FormationControlStateMachine(smach.StateMachine): 
    def __init__(self,base_ns,arm_ns):
        smach.StateMachine.__init__(self,outcomes=['formation_control_move','formation_control_stop',"formation_control_error"])
        self.__enable_manipulator=rospy.get_param("~enable_manipulator",False)
        with self:           
            smach.StateMachine.add( "Idle",
                                    States.DescisionState(  0.1,
                                                            ["~stop","~trigger","~move"],
                                                            ["stop","enable","move"]
                                                            ),
                                    transitions={   
                                                    'stop':'formation_control_stop',                                            
                                                    "move":"formation_control_move",
                                                    'enable':"EnableControl"})                                        
           
            #Enable the formation base controller
            smach.StateMachine.add('EnableControl',
                                st.FormationControlServiceState(base_ns,"slave_controller/enable_controller"),
                                transitions={'called':'FormationMovement'})

         
            if self.__enable_manipulator: 
                smach.StateMachine.add("FormationMovement",States.DescisionState(0.1,["~enable","~disable","~stop"],
                                                                                ["load_object","unload_object","stop"]),
                                                                                transitions=   {"load_object":"DriveToGrip",
                                                                                                "unload_object":"ReleaseObject",
                                                                                                "stop":"DisableControl"})    
                
                #Drive to the grip position (PTP)
                smach.StateMachine.add("DriveToGrip",
                                        mv.DrivePoseState(arm_ns,"grip"),
                                        transitions={"done":"Adjust"})
                
                #Adjust the EE position
                smach.StateMachine.add("Adjust",    st.AdjutsState(arm_ns),
                                                                transitions={"adjusted":"LinkObject"})
                
                #Link to the object (enable compliance and grip)
                smach.StateMachine.add("LinkObject", st.LinkObjectState(arm_ns),transitions={"linked":"FormationMovement"})


                #Release gripper and disable compliance
                smach.StateMachine.add("ReleaseObject", st.ReleaseObjectState(arm_ns),transitions={"released":"DriveToMove"})
                
                #Drive to move position (PTP)
                smach.StateMachine.add("DriveToMove",
                                    mv.DrivePoseState(arm_ns,"drive"),
                                    transitions={"done":"FormationMovement"})  

            else:
                smach.StateMachine.add("FormationMovement",States.DescisionState(0.1,["~stop"],
                                                                                ["stop"]),
                                                                                transitions=   {"disable":"DisableControl"}
                                                                                            )  

          
            #Disable the formation control
            smach.StateMachine.add('DisableControl',
                                    st.FormationControlServiceState(base_ns,"slave_controller/disable_controller"),
                                    transitions={'called':'Idle'})       
   

                 
           