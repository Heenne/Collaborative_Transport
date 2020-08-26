#! /usr/bin/env python
import smach
import rospy
import actionlib

from tf2_geometry_msgs import do_transform_pose
from tf import transformations
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=['initialized'],
                                    input_keys=['slaves_ns'],
                                    output_keys=['slaves_to_move'],
                                    io_keys=['slaves']
                                    )

    def execute(self,userdata):  
        rospy.loginfo("Initializing system!")                           
        userdata.slaves_to_move=userdata.slaves_ns
        userdata.slaves=dict()       
        for slave in userdata.slaves_ns:            
            userdata.slaves[slave]=dict()

        for slave in userdata.slaves_ns:
            ref=rospy.get_param("/"+slave.strip("/")+"/slave_controller/reference")
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
        return "initialized"


class InitializedState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["start","stay"])
        self.start_srv=rospy.Service("start",Empty,self.__start__)
        self.start=False
                            
    def __start__(self,req):
        self.start=True
        return True

    def execute(self,userdata):       
        if self.start:
            self.start=False
            return "start"
        else:
            rospy.Rate(10).sleep()
            return "stay"  
    
    
class CalcPosesState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["calculation_done","calculation_error"],
                                    io_keys=['slaves'])
  
    def execute(self,userdata):       
        try: 
            master_pose=PoseStamped()
            master_pose=rospy.wait_for_message("/virtual_master/master_pose",PoseStamped)
            for slave in userdata.slaves:            
                trafo=TransformStamped()
                trafo.transform.translation=master_pose.pose.position
                trafo.transform.rotation=master_pose.pose.orientation            
                slave_pose=do_transform_pose(userdata.slaves[slave]["reference"],trafo)
                slave_pose.header.frame_id="/map"
                userdata.slaves[slave]["target_pose"]=slave_pose
        except:
            return "calculation_error"
        return "calculation_done"


class CheckAllSuccededState(smach.State):
    def __init__(self):
        smach.State.__init__(self,  outcomes=["all_succeded","wait"],
                                    input_keys=['last_name'],
                                    io_keys=['slaves'])
        self.names=[]       
    def execute(self,userdata):
        if userdata.last_name not in self.names:
            self.names.append(userdata.last_name)
            
        if userdata.slaves.sort()==self.names.sort():
            return 'all_succeded'
        else:
            rospy.Rate(10).sleep()
            return 'wait'

class ErrorHandlingState(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['last_name'],outcomes=['error_handling_aborted','try_again'])
        self.retry_counter=0
    def execute(self,userdata):
        if self.retry_counter<2:
            self.retry_counter=self.retry_counter+1
            return 'try_again'
        else:  
            self.retry_counter=0          
            return "error_handling_aborted"
