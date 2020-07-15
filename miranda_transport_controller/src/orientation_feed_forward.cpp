#include<miranda_transport_controller/orientation_feed_forward.h>
#include<ros/ros.h>
OrientationFeedForward::OrientationFeedForward()
{

}


OrientationFeedForward::Pose OrientationFeedForward::updateOrientation()
{
    return this->last_pose_;
}
OrientationFeedForward::Pose OrientationFeedForward::updateOrientation(double angle)
{
    Eigen::Quaterniond quat= Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                              * Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());    
    OrientationFeedForward::Position pos=quat.inverse()*d_pos_fixed_;
    OrientationFeedForward::Pose pose;
    pose<< pos, d_ori_fixed_;
    this->last_pose_=pose;
    return pose;
}

OrientationFeedForward::Pose OrientationFeedForward::updateOrientation(Orientation ori)
{
    OrientationFeedForward::Pose pose;
    OrientationFeedForward::Position pos=ori.inverse()*d_pos_fixed_;
    pose<< pos, d_ori_fixed_;
    this->last_pose_=pose;
    return pose;
}
void OrientationFeedForward::setDesiredPose(Pose pose)
{
    d_pos_fixed_=pose.block<3,1>(0,0);
    d_ori_fixed_=pose.block<4,1>(3,0);
    d_pose_fixed_<<d_pos_fixed_,d_ori_fixed_;
    this->last_pose_=d_pose_fixed_;    
}