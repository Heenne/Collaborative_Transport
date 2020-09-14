#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped,Twist
from transport_controller.PS4 import PlayStationStateMachineHandler

if __name__=="__main__":
    rospy.init_node("ps4_state_machine_controller")
    handler=PlayStationStateMachineHandler(TwistStamped,["trigger","enable","disable","stop","move","plan"])
    handler.run()