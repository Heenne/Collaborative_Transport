#include<ros/ros.h>
#include<multi_robot_controller/feed_forward/msg_orientation_feed_forward.h>



int main(int argc, char** argv)
{
    ros::init(argc,argv,"forward_publisher");
    ros::NodeHandle nh;
    MsgOrientationFeedForward<std_msgs::Float64> feed(nh,"test_float");
    ros::spin();
}