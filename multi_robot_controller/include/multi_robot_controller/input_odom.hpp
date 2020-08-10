
#include<multi_robot_controller/input_base.hpp>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include <nav_msgs/Odometry.h>

class InputOdom: public InputBase
{   
    public:
        InputOdom(ros::NodeHandle nh,std::string topic_name)
        {
            this->sub_=nh.subscribe(topic_name,10,&InputOdom::set,this);
        }
        InputOdom(ros::NodeHandle nh)
        {
            ros::NodeHandle priv("~");
            std::string topic;
            if(!priv.getParam("topic",topic))
            {
                throw NecessaryParamException(nh.resolveName("topic"));
            }
            
            InputOdom(nh,topic);
        }
    private:
        ros::Subscriber sub_;
        inline void set(nav_msgs::Odometry msg)
        {this->setPose(msg);this->setAngVel(msg);this->setLinVel(msg);}
        inline void setPose(nav_msgs::Odometry msg) 
        {tf::poseMsgToTF(msg.pose.pose,this->pose_);}
        inline void setLinVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_);}
        inline void setAngVel(nav_msgs::Odometry msg)
        {tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_);}
};