#ifndef ROS_ORIENTATION_FEED_FORWARD_BASE_H
#define ROS_ORIENTATION_FEED_FORWARD_BASE_H

#include <multi_robot_controller/feed_forward/orientation_feed_forward.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include<multi_robot_controller/necessary_param_exeption.hpp>
#include <tf2_ros/transform_listener.h>
#include <multi_robot_controller/feed_forward/msg_conversion.hpp>
#include <std_srvs/Empty.h>

/** @addtogroup FeedForward
 * @{
 */
/**
 * @brief A Orientation feed forward class. It implements parameter loading and initialisation for the Orientation Feed Forward class. 
 * 
 */
class RosOrientationFeedForwardBase:public OrientationFeedForward{
    public:
        RosOrientationFeedForwardBase(ros::NodeHandle &nh);
        virtual bool init();
    protected:
        ros::NodeHandle nh_;      
        virtual void update(const ros::TimerEvent&)=0;
        tf2_ros::TransformListener tf_listener_;
        tf2_ros::Buffer tf_buffer_;
        std::string ee_frame_id_;
        ros::Publisher pose_pub_;
        std::string tf_prefix_;
        ros::ServiceServer init_service_;
        ros::ServiceServer enable_service_;
        bool enable_;      
    private:         
        ros::Timer update_timer_;
        bool initServiceCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyResponse &res);
        bool disableServiceCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyResponse &res);
        

};
//@}
#endif