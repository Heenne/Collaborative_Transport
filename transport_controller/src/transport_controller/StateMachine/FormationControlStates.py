import smach
import FormationControlStates as st
from ServiceDistributor import ClientDistributor,ServiceConfig
import rospy
from std_srvs.srv   import Empty,EmptyRequest ,SetBool,SetBoolRequest
from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest


class FormationControlServiceState(smach.State):
    def __init__(self,namespaces,name):
        smach.State.__init__(self,outcomes=["called"])
        self.clients=ClientDistributor(namespaces,ServiceConfig(name,Empty))

    def execute(self,userdata):
        self.clients.call(EmptyRequest())
        return "called"


class FormationControlIdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["enable",'disable','stop',"error","move"])
       
        self.__enable=False
        self.__disable=False
        self.__stop=False
        self.__called=False  
        self.__move=False
    
    def execute(self,userdata):
        enable_srv=rospy.Service("~enable",Empty,self.__enableCallback__)
        disable_srv=rospy.Service("~disable",Empty,self.__disableCallback__)
        stop_srv=rospy.Service("~stop",Empty,self.__stopCallback__)
        move_srv=rospy.Service("~move",Empty,self.__moveCallback__)

        while not self.__called:
            rospy.Rate(10).sleep

        self.__called=False
        
        enable_srv.shutdown()
        disable_srv.shutdown()
        stop_srv.shutdown()
        move_srv.shutdown()
        
        if self.__enable:
            self.__enable=False
            return "enable"
        elif self.__move:
            self.__move=False
            return "move"

        elif self.__disable:
            self.__disable=False
            return "disable"

        elif self.__stop:
            self.__stop=False
            return "stop"

        else:
            return "error"


    def __enableCallback__(self,req):
        self.__enable=True
        self.__called=True
    
    def __disableCallback__(self,req):
        self.__disable=True
        self.__called=True

    def __stopCallback__(self,req):
        self.__stop=True
        self.__called=True
    
    def __moveCallback__(self,req):
        self.__move=True
        self.__called=True


class LinkObjectState(smach.State):
    def __init__(self,namespaces):
        smach.State.__init__(self,outcomes=["linked"])
        self.__clients=ClientDistributor(namespaces,ServiceConfig("grip",SetBool))

    def execute(self,userdata):
        switcher=rospy.ServiceProxy("/miranda/panda/controller_manager/switch_controller",SwitchController)
        req=SwitchControllerRequest()
        req.start_controllers=["position_joint_controller"]
        req.start_controllers=["cartesian_impedance_controller"]
        req.strictness=2
        switcher.call(req)

        grip_req=SetBoolRequest()
        grip_req.data=True
        self.__clients.call(grip_req)

        return "linked"

class ReleaseObjectState(smach.State):
    def __init__(self,namespaces):
        smach.State.__init__(self,outcomes=["released"])
        self.__clients=ClientDistributor(namespaces,ServiceConfig("grip",SetBool))

    def execute(self,userdata):
        grip_req=SetBoolRequest()
        grip_req.data=False
        self.__clients.call(grip_req)

        switcher=rospy.ServiceProxy("/miranda/panda/controller_manager/switch_controller",SwitchController)
        req=SwitchControllerRequest()
        req.start_controllers=["cartesian_impedance_controller"]
        req.start_controllers=["position_joint_controller"]
        req.strictness=1
        switcher.call(req)
     
        return "released"
