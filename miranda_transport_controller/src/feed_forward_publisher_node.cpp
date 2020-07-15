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
    ros::NodeHandle nh("panda");
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("equilibrium_pose",10);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    bool succeed=false;
    while (!succeed)
    {
        
        try{
            transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_K",
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

    pub.publish(initial);
    OrientationFeedForward::Pose pose;
    convertMsg(pose,initial);
    RosOrientationFeedForward<std_msgs::Float64> feed(nh,"test_float");
    ROS_INFO_STREAM("Initial pose: "<<pose);
    feed.setDesiredPose(pose);

    ros::Rate rate(10);
    geometry_msgs::PoseStamped current;
    while (ros::ok())
    {
        ROS_INFO_STREAM("Current pose: "<<pose);
        convertMsg(current,pose);
        pub.publish(current);
        ros::spinOnce();
        rate.sleep();
    }    

}