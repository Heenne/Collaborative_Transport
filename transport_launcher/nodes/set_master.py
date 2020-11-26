#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


if __name__ == '__main__':
    pub = rospy.Publisher('initial_pose', PoseWithCovarianceStamped, queue_size=1)
    rospy.init_node('master_setter', anonymous=True)
    
    pose=PoseWithCovarianceStamped()
    pose.header.frame_id="map"
    pose.header.stamp=rospy.Time.now()
    pose.pose.pose.position.x=rospy.get_param("~x")
    pose.pose.pose.position.y=rospy.get_param("~y")
    pose.pose.pose.position.z=rospy.get_param("~z")
    pose.pose.pose.orientation.x=rospy.get_param("~qx")
    pose.pose.pose.orientation.y=rospy.get_param("~qy")
    pose.pose.pose.orientation.z=rospy.get_param("~qz")
    pose.pose.pose.orientation.w=rospy.get_param("~qw")
    
    for i in range(2):
        pub.publish(pose)
        rospy.Rate(2).sleep()
        

     