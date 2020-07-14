#include<ros/ros.h>
#include<miranda_transport_controller/ros_orientation_feed_forward.h>
#include<geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"forward_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("equilibrium_pose",10); 
}