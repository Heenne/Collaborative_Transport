from functools import partial 
import rospy
import smach
import smach_ros
import States as st
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MoveStateMachine(smach.StateMachine):
    def __init__(self,slave_namespaces):               
        smach.StateMachine.__init__(self,outcomes=['movement_done'],input_keys=['slaves'])        
        with self: 
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

            smach.StateMachine.add('Movement',sm_con,transitions={'success':'movement_done',
                                                                  'error_occured':'ErrorHandlingState'})
            smach.StateMachine.add("ErrorHandlingState",st.ErrorHandlingState(),                                
                                    transitions={   'error_handling_aborted':'movement_done',
                                                    'try_again':'Movement'})



    def __calcGoal__(self,name,userdata,default_goal): 
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose=userdata.slaves[name]['target_pose'].pose  
        userdata.last_name=name
        return goal
        