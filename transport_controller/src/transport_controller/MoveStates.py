import smach
import rospy
import smach_ros

from transport_controller.ServiceDistributor import ClientDistributor ,ServiceConfig
from robot_teacher.srv import SetName,SetNameRequest
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_geometry_msgs import do_transform_pose
from std_srvs.srv import Empty


class ErrorHandlingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['error_handling_aborted','try_again'])
        self.retry_counter=0
    def execute(self,userdata):
        if self.retry_counter<2:
            self.retry_counter=self.retry_counter+1
            return 'try_again'
        else:  
            self.retry_counter=0          
            return "error_handling_aborted"



class CalcPosesState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["calculation_done","calculation_error",'master_pose_error'],
                                    io_keys=['slaves'])
  
    def execute(self,userdata):       
        try: 
            master_pose=PoseStamped()
            master_pose=rospy.wait_for_message("/virtual_master/master_pose",PoseStamped)
        except:
            return 'master_pose_error'

        for slave in userdata.slaves:   
            try:                    
                trafo=TransformStamped()
                trafo.transform.translation=master_pose.pose.position
                trafo.transform.rotation=master_pose.pose.orientation            
                slave_pose=do_transform_pose(userdata.slaves[slave]["reference"],trafo)
                slave_pose.header.frame_id="/map"
                userdata.slaves[slave]["target_pose"]=slave_pose
            except:            
                return "calculation_error"
        print(userdata.slaves)
        return "calculation_done"

class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["start"])
        self.start_srv=rospy.Service("~start_movement",Empty,self.__start__)
        self.start=False
                            
    def __start__(self,req):
        self.start=True
        return True

    def execute(self,userdata):       
        while not self.start:
            rospy.Rate(10).sleep()
            pass
        self.start=False
        return "start"
    
class DrivePoseState(smach.State):
    def __init__(self,namespaces,pose_name):
        self.__pose_name=pose_name
        smach.State.__init__(self,  outcomes=["done"])       
        self.__client=ClientDistributor( namespaces,
                                        ServiceConfig("move_teached/drive_to",SetName))
    def execute(self,userdata):
        req=SetNameRequest()
        req.name=self.__pose_name
        self.__client.call(req)
        rospy.sleep(4)
        return "done"