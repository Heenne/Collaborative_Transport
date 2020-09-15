from functools import partial 
import rospy

import smach
import smach_ros

import MoveStates as st
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import States

class MoveStateMachine(smach.StateMachine):
    def __init__(self,base_namespaces,arm_namespaces):               
        smach.StateMachine.__init__(self,outcomes=['movement_done',"movement_error"],input_keys=['slaves'])        
        self.__enable_manipulator=rospy.get_param("~enable_manipulator",False)
        with self:            
            if self.__enable_manipulator:
                smach.StateMachine.add('CalcPoses', st.CalcPosesState(),
                                        transitions={   'calculation_done':'DrivePose',
                                                        'calculation_error':'movement_error',
                                                        'master_pose_error':'movement_error'})
                smach.StateMachine.add("DrivePose",st.DrivePoseState(arm_namespaces,"drive"),
                                                    transitions={  'done':'Idle'})
            else:
                smach.StateMachine.add('CalcPoses', st.CalcPosesState(),
                                        transitions={   'calculation_done':'Idle',
                                                        'calculation_error':'movement_error',
                                                        'master_pose_error':'movement_error'})
            
       
                

            smach.StateMachine.add("Idle",States.WaitTriggerState(0.1,"start_moving"),
                                    transitions={   'start_moving':'Movement'}   )
            
            
        
            sm_con=self.__allocMoveConcurrency__(base_namespaces)          

            smach.StateMachine.add('Movement',sm_con,transitions={'success':'movement_done',
                                                                  'error_occured':'ErrorHandlingState'})
            
            smach.StateMachine.add("ErrorHandlingState",st.ErrorHandlingState(),                                
                                    transitions={   'error_handling_aborted':'movement_error',
                                                    'try_again':'Movement'})



    def __calcGoal__(self,name,userdata,default_goal): 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose=userdata.slaves[name]['target_pose'].pose  
        userdata.last_name=name
        return goal
    
    def __allocMoveConcurrency__(self,slave_namespaces):
        map_abort=dict()
        for name in slave_namespaces:
            map_abort[name+"/Move"]='aborted'
            map_abort[name+"/Move"]='preempted'                
    
        map_succed=dict()
        for name in slave_namespaces:
            map_succed[name+"/Move"]='succeeded'
        
        sm_con = smach.Concurrence( outcomes=['success','error_occured'],
                                    input_keys=['slaves'],
                                    output_keys=['last_name'],
                                    default_outcome='error_occured',
                                    outcome_map={'success':map_succed,
                                                'error_occured':map_abort}) 

        with sm_con:
            for name in slave_namespaces:
                smach.Concurrence.add( name+"/Move",
                                        smach_ros.SimpleActionState(    name+'/move_base', 
                                                                        MoveBaseAction,
                                                                        goal_cb = partial(self.__calcGoal__,name),
                                                                        input_keys=['slaves'],
                                                                        output_keys=['last_name']))
        return sm_con