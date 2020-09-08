#!/usr/bin/env python
import smach
import smach_ros
import rospy


from transport_controller.StateMachine    import PrepareSystemStateMachine
from transport_controller.StateMachine    import MoveStateMachine
from transport_controller.StateMachine    import FormationControlStateMachine

if __name__=="__main__":
    rospy.init_node('transport_state_machine')
    base_namespaces=rospy.get_param("~base_namespaces",["/mur/mir","/miranda/mir"])
    arm_namespaces=rospy.get_param("~arm_namespaces",["/mur/ur","/miranda/panda"])
    sm=smach.StateMachine(outcomes=["out"])
    try:
        with sm:
            smach.StateMachine.add( "PrepareMovement",
                                    PrepareSystemStateMachine(base_namespaces),
                                    transitions={   "preparation_done":"MoveToFormation",
                                                    "preparation_error":'out'})

            smach.StateMachine.add( "MoveToFormation",
                                    MoveStateMachine(base_namespaces), 
                                    transitions={   "movement_done":"FormationControl",
                                                    "movement_error":'out'})


            smach.StateMachine.add( "FormationControl",
                                    FormationControlStateMachine(base_namespaces,arm_namespaces),
                                    transitions={   'formation_control_move':"MoveToFormation",
                                                    'formation_control_stop':'out',
                                                    "formation_control_error":'out'})

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()   
        sm.execute() 
            # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()  
    
    except Exception as e:
        print (e)    