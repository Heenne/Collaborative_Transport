#include <miranda_transport_controller/ros_orientation_feed_forward.h>


template<class T>
RosOrientationFeedForward<T>::RosOrientationFeedForward(ros::NodeHandle &nh,std::string topic):OrientationFeedForward()
{
    this->nh_=nh;
    this->ori_sub_=this->nh_.subscribe(topic,10,&RosOrientationFeedForward::callbackOrientations,this);
}

template<class T>
void RosOrientationFeedForward<T>::callbackOrientations(T msg)
{
    Eigen::Quaterniond quat;
    convertMsg(quat,msg);
    this->updateOrientation(quat);
}

template class RosOrientationFeedForward<geometry_msgs::TransformStamped>;
template class RosOrientationFeedForward<geometry_msgs::PoseStamped>;
template class RosOrientationFeedForward<geometry_msgs::Pose>;
template class RosOrientationFeedForward<std_msgs::Float64>;