
#pragma once
#include<ros/ros.h>
#include <tf/tf.h>

/** \addtogroup Input 
 *  @{
 */


class InputBase
{
    public:
        inline InputBase(): pose_(tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0))),
                            lin_vel_(tf::Vector3(0.0,0.0,0.0)),
                            ang_vel_(tf::Vector3(0.0,0.0,0.0)),
                            time_(ros::Time(0))
        {;}
        inline InputBase(ros::NodeHandle &nh):InputBase(){this->nh_=nh;}
        inline tf::Pose getPose(){this->checkValues();return this->pose_;}
        inline tf::Vector3 getLinVel(){this->checkValues();return this->lin_vel_;}
        inline tf::Vector3 getAngVel(){this->checkValues();return this->ang_vel_;}
        inline ros::Time getTime(){this->checkValues();return this->time_;}
        
    protected:      
        ros::NodeHandle nh_; 

        inline void checkValues()
        {
            if( isnan(this->pose_.getRotation().x())    ||
                isnan(this->pose_.getRotation().y())    ||
                isnan(this->pose_.getRotation().z())    ||
                isnan(this->pose_.getRotation().w()))
            {
                this->pose_.setRotation(tf::createIdentityQuaternion());
            }

        }
        tf::Pose pose_;
        tf::Vector3 lin_vel_;
        tf::Vector3 ang_vel_;
        ros::Time time_;
};
