#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>



std::string tf_prefix="";
geometry_msgs::TransformStamped transformStamped;
tf2_ros::Buffer tfBuffer;
void poseCallback(const geometry_msgs::PoseConstPtr& msg){
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/map";
    transformStamped.child_frame_id = tf::resolve(tf_prefix,"odom");    
    transformStamped.transform.translation.x = msg->position.x;
    transformStamped.transform.translation.y = msg->position.y;
    transformStamped.transform.translation.z = msg->position.z;
    transformStamped.transform.rotation.x = msg->orientation.x;
    transformStamped.transform.rotation.y = msg->orientation.y;
    transformStamped.transform.rotation.z = msg->orientation.z;
    transformStamped.transform.rotation.w = msg->orientation.w;
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped base; 
   
    tf2_ros::TransformListener tfListener(tfBuffer);

    try{
        base = tfBuffer.lookupTransform(tf::resolve(tf_prefix,"base_footprint"),
                                        tf::resolve(tf_prefix,"odom"),
                                        ros::Time(0));
        
        

        tf2::Transform trafo1;
        tf2::fromMsg(base.transform,trafo1);
        tf2::Transform trafo2;
        tf2::fromMsg(transformStamped.transform,trafo2);
        transformStamped.transform=tf2::toMsg(trafo1.inverse()*trafo2);

        br.sendTransform(transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
  
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");

    ros::NodeHandle private_node("~");
    private_node.getParam("tf_prefix", tf_prefix);

        
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("robot_pose", 10, &poseCallback);

    ros::spin();
    return 0;
};