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


class AdjutsState(smach.State):
    def __init__(self,namespaces):
        smach.State.__init__(self,outcomes=["adjusted"])
        self.__switcher=ClientDistributor(namespaces,ServiceConfig("controller_manager/switch_controller",SwitchController))         
        self.__enable=False
        self.__timeout=0.1


    def execute(self,userdata):
        req=SwitchControllerRequest()
        req.stop_controllers=["position_joint_controller"]
        req.start_controllers=["cartesian_controller"]
        req.strictness=2
        self.__switcher.call(req)
        srv=rospy.Service("~trigger",Empty,self.__callback__)
        while not self.__enable:           
            rospy.sleep(self.__timeout)      
        self.__enable=False       
        
        srv.shutdown()    
        req=SwitchControllerRequest()
        req.stop_controllers=["cartesian_controller"]
        req.start_controllers=[]
        req.strictness=2 
        self.__switcher.call(req)  
        
        return "adjusted"

    def __callback__(self,req):
        self.__enable=True
        return
    

class LinkObjectState(smach.State):
    def __init__(self,namespaces):
        smach.State.__init__(self,outcomes=["linked"])
        self.__enable_orientation_ff=rospy.get_param("~enable_orientation_ff",False)
        if self.__enable_orientation_ff:
            self.__orientation_init=ClientDistributor(namespaces,ServiceConfig("orientation_ff/enable",Empty))
        
        self.__clients=ClientDistributor(namespaces,ServiceConfig("grip",SetBool))       
        self.__switcher=ClientDistributor(namespaces,ServiceConfig("controller_manager/switch_controller",SwitchController))

    def execute(self,userdata):
        if self.__enable_orientation_ff:       
            self.__orientation_init.call(EmptyRequest())        
        
        req=SwitchControllerRequest()
        req.stop_controllers=[]
        req.start_controllers=["compliance_controller"]
        req.strictness=2
        self.__switcher.call(req)

        grip_req=SetBoolRequest()
        grip_req.data=True
        self.__clients.call(grip_req)
        return "linked"


class ReleaseObjectState(smach.State):
    def __init__(self,namespaces):
        smach.State.__init__(self,outcomes=["released"])
        self.__clients=ClientDistributor(namespaces,ServiceConfig("grip",SetBool))
        self.__switcher=ClientDistributor(namespaces,ServiceConfig("controller_manager/switch_controller",SwitchController))
        self.__enable_orientation_ff=rospy.get_param("~enable_orientation_ff",False)
        if self.__enable_orientation_ff:
            self.__orientation_init=ClientDistributor(namespaces,ServiceConfig("orientation_ff/disable",Empty))

    def execute(self,userdata):
        grip_req=SetBoolRequest()
        grip_req.data=False
        self.__clients.call(grip_req)
        rospy.sleep(2)

        req=SwitchControllerRequest()
        req.stop_controllers=["compliance_controller"]
        req.start_controllers=["position_joint_controller"]
        req.strictness=2
        self.__switcher.call(req)
        if self.__enable_orientation_ff:       
            self.__orientation_init.call(EmptyRequest())        
       
     
        return "released"


   
        