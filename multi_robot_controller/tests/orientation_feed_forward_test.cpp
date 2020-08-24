#include<ros/ros.h>
#include<multi_robot_controller/feed_forward/orientation_feed_forward.h>
#include<multi_robot_controller/feed_forward/msg_orientation_feed_forward.h>
#include<geometry_msgs/TransformStamped.h>
#include<tf2_ros/transform_broadcaster.h>


int main(int argc, char** argv)
{
    OrientationFeedForward::Pose pose_desired;
    pose_desired<<1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0; 
    OrientationFeedForward feed;
    double angle=0.0;
    feed.setDesiredPose(pose_desired);
    for(int i=0;i<10;i++)
    {
        feed.updateOrientation(angle);
        std::cout<<feed.getPose()<<std::endl<<std::endl;
        angle+=M_PI/10;        
    }
}