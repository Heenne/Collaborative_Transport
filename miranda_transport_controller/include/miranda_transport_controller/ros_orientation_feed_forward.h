#ifndef ROS_ORIENTATION_FEED_FORWARD_H
#define ROS_ORIENTATION_FEED_FORWARD_H

#include <ros/ros.h>
#include <miranda_transport_controller/orientation_feed_forward.h>
#include <miranda_transport_controller/msg_conversion.hpp>

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
class RosOrientationFeedForward:public OrientationFeedForward{
    public:
        /**
         * @brief Construct a new Ros Orientation Feed Forward object
         * 
         * @param nh Nodehandle for namespace handling
         * @param topic Topic name that should be used as input source
         */
        RosOrientationFeedForward(ros::NodeHandle &nh,std::string topic);
      
    protected:
    private:
        ros::Subscriber ori_sub_;
        ros::NodeHandle nh_;
        void callbackOrientation(T msg);
     
};
//@}
#endif