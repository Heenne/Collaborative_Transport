
#include<multi_robot_controller/Input/input_base.hpp>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include <nav_msgs/Odometry.h>

class InputOdom: public InputBase
{   
    public:
        InputOdom():InputBase()
        {;}
        InputOdom(ros::NodeHandle &nh,std::string topic_name):InputBase(nh)
        {
            this->sub_=this->nh_.subscribe(topic_name,10,&InputOdom::set,this);
        }
        InputOdom(ros::NodeHandle &nh)
        {
            ros::NodeHandle priv("~");
            std::string topic;
            if(!priv.getParam("topic",topic))
            {
                throw NecessaryParamException(priv.resolveName("topic"));
            }
            InputOdom(nh,topic);
        }
        InputOdom(ros::NodeHandle &nh,ros::NodeHandle &param_nh)
        {
            std::string topic;
            if(!param_nh.getParam("topic",topic))
            {
                throw NecessaryParamException(param_nh.resolveName("topic"));
            }
            InputOdom(nh,topic);
        }
        
    private:
        ros::Subscriber sub_;        
        inline void set(nav_msgs::Odometry msg)
        {this->setPose(msg);this->setAngVel(msg);this->setLinVel(msg);this->time_=msg.header.stamp;}
        inline void setPose(nav_msgs::Odometry msg) 
        {tf::poseMsgToTF(msg.pose.pose,this->pose_);}
        inline void setLinVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_);}
        inline void setAngVel(nav_msgs::Odometry msg)
        {tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_);}
};