#include<ros/ros.h>
#include<miranda_transport_controller/ros_orientation_feed_forward.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf2_ros/transform_listener.h>
#include<tf2_ros/buffer.h>
#include<std_msgs/Float64.h>
#include<miranda_transport_controller/msg_conversion.hpp>

#include<miranda_transport_controller/ros_orientation_feed_forward.h>



int main(int argc, char** argv)
{
    
    ros::init(argc,argv,"forward_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("equilibrium_pose",10);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    RosOrientationFeedForward<std_msgs::Float64> feed(nh,"test_float");   
    geometry_msgs::TransformStamped transformStamped,offsetStamped;
    bool succeed=false;
    while (!succeed)
    {
        try{
            transformStamped = tfBuffer.lookupTransform("mir/base_link","panda/panda_hand",
                                    ros::Time(0));
                            
            succeed=true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    succeed=false;
    while (!succeed)
    {
        try{
            offsetStamped = tfBuffer.lookupTransform("mir/base_link","panda/panda_link0",
                                    ros::Time(0));
                            
            succeed=true;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    
    geometry_msgs::PoseStamped initial;
    convertMsg(initial,transformStamped);    

    OrientationFeedForward::Pose pose;
    convertMsg(pose,initial);
    ROS_INFO_STREAM("Initial pose: "<<pose);
     feed.setDesiredPose(pose);
    
    

    OrientationFeedForward::Pose offset;
    convertMsg(offset,offsetStamped);
    ROS_INFO_STREAM("Initial offset: "<<offset);
    feed.setOffset(offset);
    
    ros::Rate rate(1000);
    OrientationFeedForward::Pose control;
    geometry_msgs::PoseStamped current;
    current.header.frame_id="panda/panda_link0";
    while (ros::ok())
    {
        control=feed.getPose();
        // ROS_INFO_STREAM("Current pose: "<<control);
        convertMsg(current,control);
        pub.publish(current);
        ros::spinOnce();
        rate.sleep();
    }    

}