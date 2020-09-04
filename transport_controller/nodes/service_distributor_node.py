#!/usr/bin/env python
import rospy
from std_srvs.srv import Empty
from robot_teacher.srv import SetName
from transport_controller import ServiceDistributor ,ServiceConfig

if __name__=="__main__":
    rospy.init_node("service_distributor")
    services=[ServiceConfig("move_teached/drive_to",SetName)]
    namespaces=["mur/ur","miranda/panda"]
    service=ServiceDistributor(namespaces,services)
    rospy.spin()
