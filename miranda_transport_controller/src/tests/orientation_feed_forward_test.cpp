#include<ros/ros.h>
#include<miranda_transport_controller/orientation_feed_forward.h>
#include<miranda_transport_controller/ros_orientation_feed_forward.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/transform_broadcaster.h>


int main(int argc, char** argv)
{
    OrientationFeedForward::Pose pose_desired;
    pose_desired<<1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0; 
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
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("test_pose_ff",10);

    RosOrientationFeedForward<std_msgs::Float64> ros_feed_float(nh,"test_float");
    ros_feed_float.setDesiredPose(pose_desired);
    ros_feed_float.setOffset(Eigen::Quaterniond(1.0,0.0,0.0,0.0),Eigen::Vector3d(0.35, 0.15, 0.35));
 
    geometry_msgs::PoseStamped pose_ff;
    pose_ff.header.frame_id="panda_link0";
    static tf2_ros::TransformBroadcaster br;

    ros::Rate rate(100);
    while(ros::ok())
    {
        Eigen::Matrix<double,7,1>pose;
        pose=ros_feed_float.getPose();
        convertMsg(pose_ff.pose,pose);
        pub.publish(pose_ff);

        geometry_msgs::TransformStamped transformStamped;
        convertMsg(transformStamped,pose_ff);
        transformStamped.header.frame_id="panda_link0";
        transformStamped.child_frame_id="ee_link";
        transformStamped.header.stamp=ros::Time::now();
        br.sendTransform(transformStamped);
        
       
       
        
        ros::spinOnce();
        rate.sleep();
    }
}