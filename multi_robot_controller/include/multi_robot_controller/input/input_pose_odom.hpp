
#include <multi_robot_controller/input/input_base.hpp>
#include <multi_robot_controller/necessary_param_exeption.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>

/** \addtogroup Input 
 *  @{
 */


class InputPoseOdom: public InputBase
{   
    public:
        InputPoseOdom():InputBase()
        {;}

        InputPoseOdom(ros::NodeHandle &nh,std::string topic_name_pose,std::string topic_name_odom):InputBase(nh)
        {
            this->subscribe(topic_name_pose,topic_name_odom);
        }

        InputPoseOdom(ros::NodeHandle &nh):InputBase(nh)
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

            this->subscribe(topic_pose,topic_odom);  
        }


        InputPoseOdom(ros::NodeHandle &nh,ros::NodeHandle &nh_param):InputBase(nh)
        {

            std::string topic_pose;
            if(!nh_param.getParam("topic_pose",topic_pose))
            {
                throw NecessaryParamException(nh_param.resolveName("topic_pose")); 
            }

            std::string topic_odom;
            if(!nh_param.getParam("topic_odom",topic_odom))
            {
                throw NecessaryParamException(nh_param.resolveName("topic_odom")); 
            }

            this->subscribe(topic_pose,topic_odom);  
        }

        ~InputPoseOdom()
        {

        }
        

    private:       
        ros::Subscriber sub_pose_;
        ros::Subscriber sub_odom_;

        inline void subscribe(std::string topic_name_pose,std::string topic_name_odom)
        {
            this->sub_pose_=this->nh_.subscribe(topic_name_pose,10,&InputPoseOdom::setPose,this);
            this->sub_odom_=this->nh_.subscribe(topic_name_odom,10,&InputPoseOdom::set,this);
        }

        inline void set(nav_msgs::Odometry msg)
        {this->setAngVel(msg);this->setLinVel(msg);this->time_=msg.header.stamp;}
       
        inline void setPose(geometry_msgs::PoseStamped msg) 
        {tf::poseMsgToTF(msg.pose,this->pose_);}
        
        inline void setLinVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.linear,this->lin_vel_);}
       
        inline void setAngVel(nav_msgs::Odometry msg) 
        {tf::vector3MsgToTF(msg.twist.twist.angular,this->ang_vel_);}
};