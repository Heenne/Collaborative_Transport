
import smach
import PrepareSystemStates as st

class PrepareSystemStateMachine(smach.StateMachine):
    def __init__(self,slave_namespaces):
        smach.StateMachine.__init__(self,   outcomes=['preparation_done','preparation_error'],
                                            output_keys=["slaves"])
        with self:      

            smach.StateMachine.add('Start',st.StartState(slave_namespaces),
                                transitions={'startup_done':'Initialized',"references_error":"preparation_error"})

            smach.StateMachine.add('Initialized',st.InitializedState(),
                                    transitions={'start':'CalcPoses'})

            smach.StateMachine.add('CalcPoses', st.CalcPosesState(),
                                    transitions={'calculation_done':'preparation_done',
                                                 'calculation_error':'Initialized',
                                                 'master_pose_error':'preparation_error'})