import smach
import rospy
from functools import partial
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

        self.__enable=False
        srv.shutdown()
        return self.__outcome
    
    def __callback__(self,req):
        self.__enable=True


class DescisionState(smach.State):
    def __init__(self,timeout,service_names,outcomes): 
        smach.State.__init__(self,  outcomes=outcomes)      
       
        self.__outcomes=outcomes        
        self.__called=False
        self.__timeout=timeout
        self.__service_names=service_names

        self.__outcome=""
      
        
    def execute(self,userdata):
        services=list()
        for i,service in enumerate(self.__service_names):         
            services.append(rospy.Service(service,Empty,partial(self.__callback__,i)))
        while not self.__called:
            rospy.sleep(self.__timeout)
        
        for service in services:
            service.shutdown()

        self.__called=False      
        return self.__outcome 

    def __callback__(self,index,req):
        self.__called=True
        self.__outcome=self.__outcomes[index]