
#include<multi_robot_controller/input_base.h>
#include<message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

class InputPoseTwist: public InputBase<geometry_msgs::PoseStamped,geometry_msgs::Twist>
{   
    public:
        InputPoseTwist(ros::NodeHandle nh,std::string topic_name_pose,std::string topic_name_twist)
        {
           
            this->sub_pose_=nh.subscribe(topic_name_pose,10,&InputPoseTwist::setPose,this);
            this->sub_twist_=nh.subscribe(topic_name_twist,10,&InputPoseTwist::set,this);
        }
    private:
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_twist_;

        inline void set(geometry_msgs::Twist msg)
        {this->setAngVel(msg);this->setLinVel(msg);}
       
        inline void setPose(geometry_msgs::PoseStamped msg) override
        {tf::poseMsgToTF(msg.pose,this->pose_);}
        
        inline void setLinVel(geometry_msgs::Twist msg) override
        {tf::vector3MsgToTF(msg.linear,this->lin_vel_);}
       
        inline void setAngVel(geometry_msgs::Twist msg) override
        {tf::vector3MsgToTF(msg.angular,this->ang_vel_);}
};