#ifndef ROS_ORIENTATION_FEED_FORWARD_BASE_H
#define ROS_ORIENTATION_FEED_FORWARD_BASE_H

#include <transport_controller/orientation_feed_forward.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <transport_controller/msg_conversion.hpp>


/** @addtogroup group_feed_forward
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
    private:
        ros::Timer update_timer_;
        

};
//@}
#endif