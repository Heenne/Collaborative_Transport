#! /usr/bin/env python
import json
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs as tf2g
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
from std_srvs.srv import Empty
from tf import transformations
import tf
import tf2_ros


class SlaveReferencer:
    def __init__(self):
        rospy.init_node('slave_referencer')
        slaves_ns=rospy.get_param('~slave_namespaces')
        self.master=rospy.get_param('~master_namespace')
        if not slaves_ns or not self.master:
            raise ValueError()
        rospy.loginfo("Master")
        rospy.loginfo(self.master)
        rospy.loginfo("Slaves:")
        rospy.loginfo(slaves_ns)

        self.slaves=dict()
        self.slave_poses=dict()
        for slave in slaves_ns:
            ref=rospy.get_param("/"+slave.strip("/")+"/reference")
            ref_pose=PoseStamped()
            ref_pose.pose.position.x=ref[0]
            ref_pose.pose.position.y=ref[1]
            ref_pose.pose.position.z=0.0
            quat=transformations.quaternion_from_euler(0.0,0.0,ref[2])
            ref_pose.pose.orientation.x=quat[0]
            ref_pose.pose.orientation.y=quat[1]
            ref_pose.pose.orientation.z=quat[2]
            ref_pose.pose.orientation.w=quat[3]
            self.slaves[slave]=ref_pose
            self.slave_poses[slave]=PoseStamped()
        rospy.loginfo("Slaves:")
        rospy.loginfo(self.slaves)
        self.maser_pose=PoseStamped()

        self.set_state_srv=rospy.Service("toggle_state",Empty,self.__srvsetState__)
        
        self.state=0

        self.movement_counter=0

        self.move_base_dict=dict()
        for slave in self.slaves:                
               self.move_base_dict[slave]=actionlib.SimpleActionClient(slave+'/move_base',MoveBaseAction)

    def __calcSlaves__(self):        
        master_pose=rospy.wait_for_message("/master_pose_stamped",PoseStamped)
        for slave in self.slaves:
            print(self.slaves[slave])
            trafo=TransformStamped()
            trafo.transform.translation=master_pose.pose.position
            trafo.transform.rotation=master_pose.pose.orientation            
            slave_pose=tf2g.do_transform_pose(self.slaves[slave],trafo)
            self.slave_poses[slave]=slave_pose
    
    def __moveSlaves__(self):
        for slave in self.slaves:            
            self.__moveSlave__(slave)

    def __moveSlave__(self,name):
        rospy.loginfo("Moving slave "+name+" to:")
        rospy.loginfo(self.slave_poses[name])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose=self.slave_poses[name].pose       
        self.move_base_dict[name].send_goal(    goal,
                                                active_cb=self.__activeCallback__,
                                                feedback_cb=self.__feedBackCallback__,
                                                done_cb=self.__doneCallback__   )     

    def __activeCallback__(self):
        rospy.loginfo("Active")
    def __feedBackCallback__(self,feedback):
        rospy.loginfo("Feedback")
    def __doneCallback__(self,state,result):
        rospy.loginfo("State Result")
    def run(self):
        if self.state==0:
            pass
        elif self.state==1:
            self.__calcSlaves__()
            rospy.loginfo("Calculated:")
            rospy.loginfo(self.slave_poses)
            self.state=2
            rospy.loginfo("1->2")

        elif self.state==2:
            pass
                        
        elif self.state==3:
            if self.movement_counter==len(self.slaves):
                self.state=0
                self.movement_counter=0
                rospy.loginfo("3->0")
            else:
               self.__moveSlave__(self.slaves.keys()[self.movement_counter])
               self.movement_counter=self.movement_counter+1
               self.state=2
               rospy.loginfo("3->2")
            
    def __srvsetState__(self,req):
        rospy.loginfo(str(self.state)+"->"+str(self.state+1))
        self.state=(self.state+1)


if __name__=="__main__":
    pose1=PoseStamped()
    pose2=PoseStamped()
    pose1.pose.position.y=-0.3
    pose1.pose.orientation.w=1.0
    pose2.pose.position.y=0.3
    pose2.pose.orientation.w=1.0
    # names={"/mur/mir":pose1,"/miranda/mir":pose2}
    names={"/mur/mir":pose1}
    ref=SlaveReferencer()
    while not rospy.is_shutdown():
        ref.run()
    
    