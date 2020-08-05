#ifndef MSG_ORIENTATION_FEED_FORWARD_H
#define MSG_ORIENTATION_FEED_FORWARD_H

#include <ros/ros.h>
#include <transport_controller/ros_orientation_feed_forward_base.h>
#include <transport_controller/msg_conversion.hpp>

/** @addtogroup group_feed_forward
 * @{
 */
/**
 * @brief A Ros implementation of the OrientationFeeedForward class. It contains a ros Subscriber that listens to a specific 
 * message and uses the message data as input data for the feed forward control. 
 * 
 * @tparam T Message type the subcriber is listeneing to. A convertMsg fucntion has to be implemented for it.
 */
template<class T>
class MsgOrientationFeedForward:public RosOrientationFeedForwardBase{
    public:
        /**
         * @brief Construct a new Ros Orientation Feed Forward object
         * 
         * @param nh Nodehandle for namespace handling
         * @param topic Topic name that should be used as input source
         */
        MsgOrientationFeedForward(ros::NodeHandle &nh,std::string topic);
        bool init() override;
    protected:
        void update(const ros::TimerEvent&)override;
        
    private:
        ros::Subscriber ori_sub_;
        T current_orientation_;
        void callbackOrientation(T msg);
     
};
//@}
#endif