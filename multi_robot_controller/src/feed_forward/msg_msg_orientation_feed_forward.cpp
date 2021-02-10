#include <multi_robot_controller/feed_forward/msg_msg_orientation_feed_forward.h>


template<class T>
MsgMsgOrientationFeedForward<T>::MsgMsgOrientationFeedForward(ros::NodeHandle &nh,std::string topic):RosOrientationFeedForwardBase(nh)                                                         
{
    this->target_ori_sub_=this->nh_.subscribe("target_orientation",10,&MsgMsgOrientationFeedForward::callbackCurrentOrientation,this);
    this->current_ori_sub_=this->nh_.subscribe("current_orientation",10,&MsgMsgOrientationFeedForward::callbackTargetOrientation,this);
   
}

template<class T>
void MsgMsgOrientationFeedForward<T>::callbackCurrentOrientation(T msg)
{
   this->current_orientation_ =msg;
}
template<class T>
void MsgMsgOrientationFeedForward<T>::callbackTargetOrientation(T msg)
{
   this->target_orientation_ =msg;
}
template<class T>
void MsgMsgOrientationFeedForward<T>::update(const ros::TimerEvent&)
{
    if(this->enable_)
    {
        Eigen::Quaterniond quat_current;
        convertMsg(quat_current,this->current_orientation_);
        Eigen::Quaterniond quat_target;
        convertMsg(quat_target,this->target_orientation_);

        this->updateOrientation(quat_current.inverse()*quat_target);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id=tf::resolve(this->tf_prefix_,this->ee_frame_id_);
        Pose forward=this->getPose();
        convertMsg(pose,forward);
        this->pose_pub_.publish(pose);

    }
   
}
template<class T>
bool MsgMsgOrientationFeedForward<T>::init()
{
    ros::NodeHandle nh("~");
    RosOrientationFeedForwardBase::init();
    this->target_orientation_=*ros::topic::waitForMessage<T>("target_orientation",this->nh_);
    this->current_orientation_=*ros::topic::waitForMessage<T>("current_orientation",this->nh_);
    Eigen::Quaterniond quat_current;
    convertMsg(quat_current,this->current_orientation_);
    Eigen::Quaterniond quat_target;
    convertMsg(quat_target,this->target_orientation_);
    this->setInitial(quat_current.inverse()*quat_target);
   
}

template class MsgMsgOrientationFeedForward<geometry_msgs::TransformStamped>;
template class MsgMsgOrientationFeedForward<geometry_msgs::PoseStamped>;
template class MsgMsgOrientationFeedForward<geometry_msgs::Pose>;
template class MsgMsgOrientationFeedForward<std_msgs::Float64>;