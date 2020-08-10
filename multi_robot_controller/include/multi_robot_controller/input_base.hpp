
#include<ros/ros.h>
#include <tf/tf.h>



class InputBase
{
    public:
        tf::Pose getPose(){return this->pose_;}
        tf::Vector3 getLinVel(){return this->lin_vel_;}
        tf::Vector3 getAngVel(){return this->ang_vel_;}
    protected:
        ros::Subscriber sub_pose;
        ros::Subscriber sub_lin_;
        ros::Subscriber sub_ang_;
        
        tf::Pose pose_;
        tf::Vector3 lin_vel_;
        tf::Vector3 ang_vel_;
        // virtual void load()=0;
};
