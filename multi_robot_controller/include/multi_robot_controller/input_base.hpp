
#pragma once
#include<ros/ros.h>
#include <tf/tf.h>



class InputBase
{
    public:
        inline InputBase(): pose_(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0))),
                            lin_vel_(tf::Vector3(0.0,0.0,0.0)),
                            ang_vel_(tf::Vector3(0.0,0.0,0.0)),
                            time_(ros::Time(0))
        {;}
        inline InputBase(ros::NodeHandle &nh):InputBase(){this->nh_=nh;}
        inline tf::Pose getPose(){return this->pose_;}
        inline tf::Vector3 getLinVel(){return this->lin_vel_;}
        inline tf::Vector3 getAngVel(){return this->ang_vel_;}
        inline ros::Time getTime(){return this->time_;}
        
    protected:      
        ros::NodeHandle nh_; 

        tf::Pose pose_;
        tf::Vector3 lin_vel_;
        tf::Vector3 ang_vel_;
        ros::Time time_;
};
