#!/usr/bin/env python
import smach
import smach_ros
import rospy

from transport_controller.PrepareSystemStateMachine import PrepareSystemStateMachine
from transport_controller.MoveStateMachine  import MoveStateMachine


if __name__=="__main__":
    rospy.init_node('transport_state_machine')
 
    sm=smach.StateMachine(outcomes=["out"])
    
    prepare=PrepareSystemStateMachine(["/mur/mir","/miranda/mir"])
    movement=MoveStateMachine(["/mur/mir","/miranda/mir"])
    with sm:
        smach.StateMachine.add("Prepare_Movement",prepare,   transitions={"preparation_done":"Move_Slave","preparation_error":'out'})
        smach.StateMachine.add("Move_Slave",movement,   transitions={"movement_done":"Prepare_Movement"})
          


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

   
    

    