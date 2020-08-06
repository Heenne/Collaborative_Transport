
#include<ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>


template <class P,class A=P,class L=A>
class InputBase
{
    public:
        virtual void setPose(P value)=0;
        virtual void setAngVel(A value)=0;
        virtual void setLinVel(L value)=0;
        tf::Pose getPose(){return this->pose_;}
        tf::Vector3 getLinVel(){return this->lin_vel_;}
        tf::Vector3 getAngVel(){return this->ang_vel_;}
    protected:
        tf::Pose pose_;
        tf::Vector3 lin_vel_;
        tf::Vector3 ang_vel_;
};
