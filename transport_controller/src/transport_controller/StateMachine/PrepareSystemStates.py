import smach
import rospy
from tf import transformations

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty,EmptyRequest,SetBool,SetBoolRequest
from ServiceDistributor import ClientDistributor,ServiceConfig
from controller_manager_msgs.srv import SwitchController,SwitchControllerRequest


class StartState(smach.State):
    def __init__(self,base_namespaces,arm_namespaces):
        smach.State.__init__(self,  outcomes=['startup_done',"references_error"],
                                        io_keys=['slaves']
                                        )
        self.__enable_manipulator=rospy.get_param("~enable_manipulator",False)
        self.__arm_controller=self.__switcher=ClientDistributor(arm_namespaces,ServiceConfig("controller_manager/switch_controller",SwitchController))   
        self.__arm_namespaces=arm_namespaces
        self.__base_namespaces=base_namespaces
       
        self.__arm_request=SwitchControllerRequest()
        self.__arm_request.start_controllers=["position_joint_controller"]
        self.__arm_request.strictness=2
        
        if self.__enable_manipulator:
            self.__gripper_clients=ClientDistributor(arm_namespaces,ServiceConfig("grip",SetBool)) 

        self.__formation_disabler=ClientDistributor(base_namespaces,ServiceConfig("slave_controller/disable_controller",Empty))


    def execute(self,userdata):
        userdata.slaves=dict()     
        for base in self.__base_namespaces:            
            userdata.slaves[base]=dict()
            #Load reference vectors as poses
            try:
                ref=rospy.get_param(base+"/slave_controller/reference")
                ref_pose=PoseStamped()
                ref_pose.pose.position.x=ref[0]
                ref_pose.pose.position.y=ref[1]
                ref_pose.pose.position.z=0.0
                quat=transformations.quaternion_from_euler(0.0,0.0,ref[2])
                ref_pose.pose.orientation.x=quat[0]
                ref_pose.pose.orientation.y=quat[1]
                ref_pose.pose.orientation.z=quat[2]
                ref_pose.pose.orientation.w=quat[3]
                userdata.slaves[base]["reference"]=ref_pose
            except:
                return "references_error"
        if self.__enable_manipulator:
            req=SetBoolRequest()
            req.data=False
            self.__gripper_clients.call(req)
            self.__formation_disabler.call(EmptyRequest())
            self.__arm_controller.call(self.__arm_request)
        return "startup_done"