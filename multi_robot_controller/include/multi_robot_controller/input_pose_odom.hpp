
#include <multi_robot_controller/input_base.hpp>
#include <multi_robot_controller/necessary_param_exeption.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

class InputPoseOdom: public InputBase
{   
    public:
        inline InputPoseOdom(ros::NodeHandle nh,std::string topic_name_pose,std::string topic_name_odom)
        {
            // this->sub_pose_.subscribe(nh,topic_name_pose,10);
            // this->sub_odom_.subscribe(nh,topic_name_odom,10);
            // this->sync_=new message_filters::TimeSynchronizer<nav_msgs::Odometry,geometry_msgs::PoseStamped>(this->sub_odom_,this->sub_pose_,10);
            // this->sync_->registerCallback(&InputPoseOdom::sync,this);              
            this->sub_pose_=nh.subscribe(topic_name_pose,10,&InputPoseOdom::setPose,this);
            this->sub_odom_=nh.subscribe(topic_name_odom,10,&InputPoseOdom::set,this);
        }
        inline InputPoseOdom(ros::NodeHandle nh)
        {
            ros::NodeHandle priv("~");
            std::string topic_pose;
            if(!priv.getParam("topic_pose",topic_pose))
            {
              throw NecessaryParamException(priv.resolveName("topic_pose")); 
            }
            std::string topic_odom;
            if(!priv.getParam("topic_odom",topic_odom))
            {
                throw NecessaryParamException(priv.resolveName("topic_odom")); 
            }
            InputPoseOdom(nh,topic_pose,topic_odom);
        }
        ~InputPoseOdom()
        {
            // delete this->sync_;
        }
    private:
    //     message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;
    //     message_filters::Subscriber<nav_msgs::Odometry> sub_odom_;
    //     message_filters::TimeSynchronizer<nav_msgs::Odometry,geometry_msgs::PoseStamped>* sync_;
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_odom_;

        // inline void sync(const nav_msgs::OdometryPtr &msg_odom,const geometry_msgs::PoseStampedPtr &msg_pose)
        // {this->set(*msg_odom);this->setPose(*msg_pose);}
     
        inline void set(nav_msgs::Odometry msg)
        {this->setAngVel(msg);this->setLinVel(msg);}
       
        inline void setPose(geometry_msgs::PoseStamped msg) 
        {tf::poseMsgToTF(msg.pose,this->pose_);}
        
        inline void setLinVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_);}
       
        inline void setAngVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_);}
};