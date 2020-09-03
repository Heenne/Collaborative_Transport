import smach
import FormationControlStates as st
import rospy
from std_srvs.srv   import Empty,EmptyRequest


class FormationControlServiceState(smach.State):
    def __init__(self,namespaces,service_name):
        smach.State.__init__(self,outcomes=["called"])
        self.clients=list()
        for name in namespaces:
            self.clients.append(rospy.ServiceProxy(name+'/'+service_name, Empty))


    def execute(self,userdata):
        for client in self.clients:
            client.call(EmptyRequest())
        return "called"


class FormationControlIdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=["enable",'disable','stop',"error"])
        self.enable_srv=rospy.Service("~enable",Empty,self.__enableCallback__)
        self.disable_srv=rospy.Service("~disable",Empty,self.__disableCallback__)
        self.stop_srv=rospy.Service("~stop_control",Empty,self.__stopCallback__)
        self.enable=False
        self.disable=False
        self.stop=False
        self.called=False
    
    def __enableCallback__(self,req):
        self.enable=True
        self.called=True
    
    def __disableCallback__(self,req):
        self.disable=True
        self.called=True

    def __stopCallback__(self,req):
        self.stop=True
        self.called=True
    
    def execute(self,userdata):
        while not self.called:
            rospy.Rate(10).sleep
        self.called=False
        if self.enable:
            self.enable=False
            return "enable"
        elif self.disable:
            self.disable=False
            return "disable"
        elif self.stop:
            self.stop=False
            return "stop"
        else:
            return "error"


    
