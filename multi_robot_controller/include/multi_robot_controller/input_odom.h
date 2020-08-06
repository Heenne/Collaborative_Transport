
#include<multi_robot_controller/input_base.h>
#include <nav_msgs/Odometry.h>

class InputOdom: public InputBase<nav_msgs::Odometry>
{   
    public:
        InputOdom(ros::NodeHandle nh,std::string topic_name)
        {
            this->sub_=nh.subscribe(topic_name,10,&InputOdom::set,this);
        }
    private:
        ros::Subscriber sub_;
        inline void set(nav_msgs::Odometry msg)
        {this->setPose(msg);this->setAngVel(msg);this->setLinVel(msg);}
        inline void setPose(nav_msgs::Odometry msg) override
        {tf::poseMsgToTF(msg.pose.pose,this->pose_);}
        inline void setLinVel(nav_msgs::Odometry msg) override
        {tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_);}
        inline void setAngVel(nav_msgs::Odometry msg) override
        {tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_);}
};