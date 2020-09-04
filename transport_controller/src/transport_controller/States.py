import smach
import rospy
from std_srvs.srv import Empty

class WaitTriggerState(smach.State):
    def __init__(self,timeout,outcome):
        smach.State.__init__(self,  outcomes=[outcome])       
        self.__enable=False
        self.__timeout=timeout
        self.__outcome=outcome

    def execute(self,userdata):
        srv=rospy.Service("~trigger",Empty,self.__callback__)
        
        while not self.__enable:
            rospy.sleep(self.__timeout)

        self.enable=False
        srv.shutdown()
        return self.__outcome
    
    def __callback__(self,req):
        self.__enable=True
