#ifndef MSG_ORIENTATION_FEED_FORWARD_H
#define MSG_ORIENTATION_FEED_FORWARD_H

#include <ros/ros.h>
#include <multi_robot_controller/feed_forward/ros_orientation_feed_forward_base.h>
#include <multi_robot_controller/feed_forward/msg_conversion.hpp>

/** @addtogroup FeedForward
 * @{
 */
/**
 * @brief A Ros implementation of the OrientationFeeedForward class. It contains a ros Subscriber that listens to a specific 
 * message and uses the message data as input data for the feed forward control. 
 * 
 * @tparam T Message type the subcriber is listeneing to. A convertMsg fucntion has to be implemented for it.
 */
template<class T>
class MsgMsgOrientationFeedForward:public RosOrientationFeedForwardBase{
    public:
        /**
         * @brief Construct a new Ros Orientation Feed Forward object
         * 
         * @param nh Nodehandle for namespace handling
         * @param topic Topic name that should be used as input source
         */
        MsgMsgOrientationFeedForward(ros::NodeHandle &nh,std::string topic);
        bool init() override;
    protected:
        void update(const ros::TimerEvent&)override;
        
    private:
        ros::Subscriber current_ori_sub_;
        ros::Subscriber target_ori_sub_;
        std::string topic_;
        T current_orientation_;
        T target_orientation_;
        void callbackTargetOrientation(T msg);
        void callbackCurrentOrientation(T msg);
     
};
//@}
#endif