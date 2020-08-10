
#include<ros/ros.h>
#include <tf/tf.h>



class InputBase
{
    public:
        inline tf::Pose getPose(){return this->pose_;}
        inline tf::Vector3 getLinVel(){return this->lin_vel_;}
        inline tf::Vector3 getAngVel(){return this->ang_vel_;}
        inline ros::Time getTime(){return this->time_;}
    protected:
        ros::Subscriber sub_pose;
        ros::Subscriber sub_lin_;
        ros::Subscriber sub_ang_;
        
        tf::Pose pose_;
        tf::Vector3 lin_vel_;
        tf::Vector3 ang_vel_;
        ros::Time time_;
};
