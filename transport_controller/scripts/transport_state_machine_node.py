#!/usr/bin/env python
import smach
import smach_ros
import rospy

from transport_controller.PrepareSystemStateMachine import PrepareSystemStateMachine
from transport_controller.MoveStateMachine  import MoveStateMachine
from transport_controller.FormationControlStateMachine import FormationControlStateMachine

if __name__=="__main__":
    rospy.init_node('transport_state_machine')
 
    sm=smach.StateMachine(outcomes=["out"])
    namespaces=["/mur/mir","/miranda/mir"]
    with sm:
        smach.StateMachine.add( "PrepareMovement",
                                PrepareSystemStateMachine(namespaces),
                                transitions={   "preparation_done":"MoveToFormation",
                                                "preparation_error":'out'})

        smach.StateMachine.add( "MoveToFormation",
                                MoveStateMachine(namespaces), 
                                transitions={   "movement_done":"FormationControl",
                                                "movement_error":'out'})


        smach.StateMachine.add( "FormationControl",
                                FormationControlStateMachine(namespaces),
                                transitions={   'formation_control_done':'PrepareMovement',
                                                "formation_control_error":'out'})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

   
    

    