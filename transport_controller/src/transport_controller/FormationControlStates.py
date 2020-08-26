import smach
import FormationControlStates as st
import rospy
from std_srvs.srv   import Empty

class FormationControlServiceState(smach.State):
    def __init__(self,namespaces,service_name):
        smach.State.__init__(outcomes=["called"])
        self.clients=list()
        self.service=rospy.Service("~"+service_name,Empty,self.__callback__)
        for name in namespaces:
            self.clients.append(rospy.ServiceProxy(name+'/'+service_name, Empty))
        self.called=False

    def execute(self,userdata):
        while not self.called:
            rospy.Rate(10).sleep()
        for client in self.clients:
            client.call(Empty())
        self.called=False
        return "called"
    def __callback__(self,req):
        self.called=True
        return True

class FormationControlIdleState(smach.State):
    def __init__(self):
        smach.State.__init__(outcomes=["enable",'disable','stop'])
        pass