#!/usr/bin/env python
import transport_controller.States as st
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from transport_controller.PrepareSystemStateMachine import PrepareSystemStateMachine
from transport_controller.MoveStateMachine  import MoveStateMachine
import rospy
import smach
import smach_ros


if __name__=="__main__":
    rospy.init_node('transport_state_machine')
 
    sm=smach.StateMachine(outcomes=["out"])
    sm.userdata.slaves_ns=["/mur/mir","miranda/mir"]
    prepare=PrepareSystemStateMachine()
    movement=MoveStateMachine(["/mur/mir","miranda/mir"])
    with sm:
        smach.StateMachine.add("Prepare_Movement",prepare,   transitions={"preparation_done":"Move_Slave"})
        smach.StateMachine.add("Move_Slave",movement,   transitions={"movement_done":"Prepare_Movement"})
          


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

   
    

    