import rospy
from functools import partial
import actionlib

class ClientDistributor():
    def __init__(self,namespaces,service):
        self.clients=list()
        for name in namespaces:
            rospy.loginfo("Adding Client:")
            rospy.loginfo(name+"/"+service.name)
            rospy.loginfo("Of type:")
            rospy.loginfo(service.type)
            client=rospy.ServiceProxy(name+"/"+service.name,service.type)
            self.clients.append(client)
    
    def call(self,req):
        for client in self.clients:
            client.call(req)       

class ServiceDistributor():
    def __init__(self,namespaces,service):
        self.clients=ClientDistributor(namespaces,service)            
        rospy.loginfo("Adding Service:")
        rospy.loginfo(service.name)
        rospy.loginfo("Of type:")
        rospy.loginfo(service.type)
        self.__service=rospy.Service(service.name,service.type,partial(self.__serviceCallback__))            
    
    def __serviceCallback__(self,req):
        self.clients.call(req)
        

    
class ServiceConfig():
    def __init__(self,name,type):
        self.name=name
        self.type=type





###NOT TESTED!
class ActionServerDistributor():
    def __init__(self,namespaces,service):
        self.__feedbacks={name:None for name in namespaces}
        self.__dones={name:None for name in namespaces}
        self.__actives={name:None for name in namespaces}    

        
        clients=list()
        for name in namespaces:
            rospy.loginfo("Adding Client:")
            rospy.loginfo(name+"/"+service.name)
            rospy.loginfo("Of type:")
            rospy.loginfo(service.type)

            client=actionlib.SimpleActionClient(name+"/"+service.name,service.type)
            client.done_cb=partial(self.__activeCallback__,name)
            client.feedback_cb=partial(self.__doneCallback__,name)
            client.active_cb=partial(self.__feedbackCallback__,name)
            clients.append(client)

        rospy.loginfo("Server:")
        rospy.loginfo(service.name)
        rospy.loginfo("Of type:")
        rospy.loginfo(service.type)
        self.server=actionlib.SimpleActionServer(service.name,service.type,partial(self.__goalCallback__))
        
        def __activeCallback__(self,name):
            self.__actives[name]=True

        def __doneCallback__(self,name,state,result):
            self.__dones[name][state]=state
            self.__dones[name][result]=result           

        def __feedbackCallback__(self,name,feedback):
            self.__feedbacks[name]=feedback

        def __goalCallback__(self,goal):
            for client in self.__clients:
                client.send_goal(goal)
            rate=rospy.Rate(10)
            while not all(done for done in dones.values()):
                self.__publishFeedbacks__()
                rate.sleep()
            self.server.set_succeded()

        def __publishFeedbacks__(self):
            self.server.publish_feedback(self.feedbacks.values()[self.__feedback_counter])
            
                
      
