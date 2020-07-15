#ifndef ROS_ORIENTATION_FEED_FORWARD_H
#define ROS_ORIENTATION_FEED_FORWARD_H

#include <ros/ros.h>
#include <miranda_transport_controller/orientation_feed_forward.h>
#include <miranda_transport_controller/msg_conversion.hpp>


template<class T>
class RosOrientationFeedForward:public OrientationFeedForward{
    public:
        RosOrientationFeedForward(ros::NodeHandle &nh,std::string topic);
      
    protected:
    private:
        ros::Subscriber ori_sub_;
        ros::NodeHandle nh_;
        void callbackOrientation(T msg);
     
};
#endif