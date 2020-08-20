#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs as tf2g
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty



class SlaveReferencer:
    def __init__(self,ref_dict,master):
        rospy.init_node('slave_referencer')
        self.slave_names=ref_dict.keys()
        self.reference_dict=ref_dict
        self.slave_poses=dict((el,PoseStamped) for el in self.slave_names)    
        
        self.master_sub=rospy.Subscriber("master_pose",PoseStamped,self.__masterCallback__)
        self.maser_pose=PoseStamped()

        self.set_state_srv=rospy.Service("toggle_state",Empty,self.__srvsetState__)
        
        self.state=0

        self.movement_counter=0

        self.move_base_dict=dict()
        for name in self.slave_names:                
               self.move_base_dict[name]=actionlib.SimpleActionClient(name+'/move_base',MoveBaseAction)
    
    def __masterCallback__(self,msg):
        self.maser_pose=msg

    def __calcSlaves__(self):        
        master_pose=rospy.wait_for_message("/master_pose_stamped",PoseStamped)
        for name in self.slave_names:
            trafo=TransformStamped()
            trafo.transform.translation=master_pose.pose.position
            trafo.transform.rotation=master_pose.pose.orientation            
            slave_pose=tf2g.do_transform_pose(self.reference_dict[name],trafo)
            self.slave_poses[name]=slave_pose
        
    
    def __moveSlaves__(self):
        for name in self.slave_names:            
            self.__moveSlave__(name)

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
            if self.movement_counter==len(self.slave_names):
                self.state=0
                self.movement_counter=0
                rospy.loginfo("3->0")
            else:
               self.__moveSlave__(self.slave_names[self.movement_counter])
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
    ref=SlaveReferencer(names,"virtual_master")
    while not rospy.is_shutdown():
        ref.run()
    
    