#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import turtlesim.msg

tf_prefix=''
def callbackPose(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/map"
    t.child_frame_id = tf_prefix+"/base_link"
    t.transform.translation=msg.pose.position
    t.transform.rotation=msg.pose.orientation   
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_turtle_broadcaster')
    global tf_prefix
    tf_prefix=rospy.get_param('~tf_prefix')
    rospy.Subscriber(   "robot_pose",
                        PoseStamped,
                        callbackPose)
    rospy.spin()