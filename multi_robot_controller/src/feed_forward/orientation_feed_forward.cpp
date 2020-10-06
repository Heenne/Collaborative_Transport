#include<multi_robot_controller/feed_forward/orientation_feed_forward.h>
#include<ros/ros.h>
OrientationFeedForward::OrientationFeedForward()
{
    d_pos_fixed_<<1,0,0;

    d_ori_fixed_.setIdentity();
    this->current_ori_.setIdentity();
    this->initial_ori_.setIdentity();
    this->offset_rotation_.setIdentity();
    this->offset_.setIdentity();
    this->desired_ee_.setIdentity();

    this->offset_position_<<0.0,0.0,0.0;
    
}
OrientationFeedForward::OrientationFeedForward(Position pos_off,Orientation ori_off):OrientationFeedForward()
{
    this->setOffset(pos_off,ori_off);
}
void OrientationFeedForward::setOffset(Position pos_off, Orientation ori_off)
{
    this->offset_.setIdentity();
    this->offset_.translate(pos_off);
    this->offset_.rotate(ori_off);
}
void OrientationFeedForward::setInitial(Orientation initial_ori)
{
    this->initial_ori_=initial_ori_;
}
void OrientationFeedForward::setOffset(Pose pose)
{
    this->setOffset(Position(pose.block<3,1>(0,0)),Orientation(pose.block<4,1>(3,0)));
}

void OrientationFeedForward::setDesiredPose(Pose pose)
{
    this->setDesiredPose(Position(pose.block<3,1>(0,0)),Orientation(pose.block<4,1>(3,0))); 
}

void OrientationFeedForward::setDesiredPose(Position position,Orientation orientation)
{    
    this->desired_ee_.setIdentity();    
    this->desired_ee_.translate(position);
    this->desired_ee_.rotate(orientation);
}

void OrientationFeedForward::updateOrientation(double angle)
{
    Eigen::Quaterniond ori= Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
    this->updateOrientation(ori);
}

void OrientationFeedForward::updateOrientation(Orientation ori)
{
    this->current_.setIdentity();
    this->current_.rotate(ori);
    this->initial_ori_.inverse()*this->current_;
}

OrientationFeedForward::Pose OrientationFeedForward::getPose()
{
    Pose pose;
    pose<<transformPosition(),transformOrientation().coeffs();   
    return pose;    
}
OrientationFeedForward::Position OrientationFeedForward::transformPosition()
{
   return (this->offset_.inverse()*this->current_.inverse()*this->desired_ee_).translation();
}
OrientationFeedForward::Orientation OrientationFeedForward::transformOrientation()
{
    return Eigen::Quaterniond((this->offset_.inverse()*this->current_.inverse()*this->desired_ee_).rotation());
}
