import smach
import rospy
from tf import transformations

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class StartState(smach.State):
    def __init__(self,slave_namespaces):
        smach.State.__init__(self,  outcomes=['startup_done',"references_error"],
                                    io_keys=['slaves']
                                    )
        self.slave_namespaces=slave_namespaces

    def execute(self,userdata):  
        rospy.loginfo("Initializing system!")                           
        userdata.slaves=dict()       
        for slave in self.slave_namespaces:            
            userdata.slaves[slave]=dict()
            #Load reference vectors as poses
            try:
                ref=rospy.get_param(slave+"/slave_controller/reference")
                ref_pose=PoseStamped()
                ref_pose.pose.position.x=ref[0]
                ref_pose.pose.position.y=ref[1]
                ref_pose.pose.position.z=0.0
                quat=transformations.quaternion_from_euler(0.0,0.0,ref[2])
                ref_pose.pose.orientation.x=quat[0]
                ref_pose.pose.orientation.y=quat[1]
                ref_pose.pose.orientation.z=quat[2]
                ref_pose.pose.orientation.w=quat[3]
                userdata.slaves[slave]["reference"]=ref_pose
            except:
                return "references_error"
        return "startup_done"


class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["start"])
        self.start_srv=rospy.Service("~start",Empty,self.__start__)
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