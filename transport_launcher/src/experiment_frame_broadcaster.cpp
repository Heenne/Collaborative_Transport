#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>


tf2_ros::Buffer tfBuffer;


bool srvReset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("map", "base_footprint",
                            ros::Time(0));
    
        transformStamped.header.frame_id="map";
        transformStamped.child_frame_id="experiment";

        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        static_broadcaster.sendTransform(transformStamped);   
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"experiment_resetter");
    ros::NodeHandle nh; 
    ros::ServiceServer service = nh.advertiseService("reset_experiment",srvReset);
    ros::spin();

}