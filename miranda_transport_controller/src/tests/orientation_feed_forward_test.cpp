#include<ros/ros.h>
#include<miranda_transport_controller/orientation_feed_forward.h>
#include<miranda_transport_controller/ros_orientation_feed_forward.h>
#include<geometry_msgs/TransformStamped.h>
int main(int argc, char** argv)
{
    OrientationFeedForward::Pose pose_desired;
    pose_desired<<1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0; 
    OrientationFeedForward feed(Eigen::Quaterniond(1.0,0.0,0.0,0.0),Eigen::Vector3d(1.0,0.0,0.0));
    double angle=0.0;
    feed.setDesiredPose(pose_desired);
    for(int i=0;i<10;i++)
    {
        feed.updateOrientation(angle);
        // std::cout<<feed.getPose()<<std::endl<<std::endl;
        angle+=M_PI/10;        
    }

    ros::init(argc,argv,"ros_orientation_feed_forward_test");
    ros::NodeHandle nh;
    RosOrientationFeedForward<geometry_msgs::TransformStamped> ros_feed_trafo(nh,"test_trafo");
    ros_feed_trafo.setDesiredPose(pose_desired);
    RosOrientationFeedForward<geometry_msgs::PoseStamped> ros_feed_pose(nh,"test_pose");
    ros_feed_pose.setDesiredPose(pose_desired);
    RosOrientationFeedForward<std_msgs::Float64> ros_feed_float(nh,"test_float");
    ros_feed_float.setDesiredPose(pose_desired);
    ros_feed_float.setOffset(Eigen::Quaterniond(1.0,0.0,0.0,0.0),Eigen::Vector3d(1.0,0.0,0.0));
    ros::Rate rate(1);
    while(ros::ok())
    {
        std::cout<<"pose"<<std::endl<<ros_feed_float.getPose()<<std::endl<<std::endl;
        ros::spinOnce();
        rate.sleep();
    }
}