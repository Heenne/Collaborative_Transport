#include <miranda_transport_controller/tf_orientation_feed_forward.h>

TfOrientationFeedForward::TfOrientationFeedForward(ros::NodeHandle &nh):RosOrientationFeedForwardBase(nh)                                                
{
}
void TfOrientationFeedForward::update(const ros::TimerEvent&)
{
    try{
        geometry_msgs::TransformStamped trafo=tf_buffer_.lookupTransform(this->current_source_frame_,this->current_target_frame_,ros::Time(0));
        Orientation quat;
        convertMsg(quat,trafo);
        this->updateOrientation(quat);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id=this->ee_frame_id_;
        Pose forward=this->getPose();
        convertMsg(pose,forward);
        this->pose_pub_.publish(pose);    
    }
    catch(tf2::TransformException &ex) {
                ROS_WARN("Could NOT find trafo for initial pose lookupfrom %s to %s: %s",
                                                        this->current_source_frame_.c_str(),
                                                        this->current_target_frame_.c_str(), ex.what());
    }
}
bool TfOrientationFeedForward::init()
{
    RosOrientationFeedForwardBase::init();
    ros::NodeHandle nh("~");
    if(!nh.getParam("current_target_frame",this->current_target_frame_)||!nh.getParam("current_source_frame",this->current_source_frame_))
    {
        ROS_WARN("Frames for current orientation contain error!");
    }
}