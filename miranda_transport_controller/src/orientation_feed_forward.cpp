#include<miranda_transport_controller/orientation_feed_forward.h>
#include<ros/ros.h>
OrientationFeedForward::OrientationFeedForward()
{
    d_pos_fixed_<<1,0,0;

    d_ori_fixed_.setIdentity();
    this->current_ori_.setIdentity();
    this->offset_rotation_.setIdentity();
    this->offset_.setIdentity();
    this->desired_ee_.setIdentity();

    this->offset_position_<<0.0,0.0,0.0;
    
}
OrientationFeedForward::OrientationFeedForward(Orientation ori_off,Position pos_off):OrientationFeedForward()
{
    setOffset(ori_off,pos_off);
}

void OrientationFeedForward::setOffset(Pose pose)
{
    // offset_rotation_=Eigen::Quaterniond(pose.block<4,1>(3,0));
    // offset_position_=pose.block<3,1>(0,0);
    

    this->offset_.setIdentity();
    this->offset_.rotate(Eigen::Quaterniond(pose.block<4,1>(3,0)));
    this->offset_.translate(pose.block<3,1>(0,0));
}
void OrientationFeedForward::setOffset(Orientation ori_off,Position pos_off)
{
    // offset_rotation_=ori_off;
    // offset_position_=pos_off;

    this->offset_.setIdentity();
    this->offset_.rotate(ori_off);
    this->offset_.translate(pos_off);
}

void OrientationFeedForward::setDesiredPose(Pose pose)
{
    // d_pos_fixed_=pose.block<3,1>(0,0);
    // d_ori_fixed_=Eigen::Quaterniond(pose.block<4,1>(3,0));


    this->desired_ee_.setIdentity();
    this->desired_ee_.rotate(Eigen::Quaterniond(pose.block<4,1>(3,0)));
    this->desired_ee_.translate(pose.block<3,1>(0,0));
    
}

void OrientationFeedForward::setDesiredPose(Position position,Orientation orientation)
{
    // d_pos_fixed_=position;
    // d_ori_fixed_=orientation;
    
    this->desired_ee_.setIdentity();
    this->desired_ee_.rotate(orientation);
    this->desired_ee_.translate(position);
}

void OrientationFeedForward::updateOrientation(double angle)
{
    Eigen::Quaterniond ori= Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                            * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
   this->current_ori_=ori;
}

void OrientationFeedForward::updateOrientation(Orientation ori)
{
    this->current_=ori;
    current_.setIdentity();
    current_.rotate(ori);
}

OrientationFeedForward::Pose OrientationFeedForward::getPose()
{
    Position pos=transformPosition();
    Pose pose;
    pose<<transformPosition(),transformOrientation().coeffs();   
    return pose;    
}
OrientationFeedForward::Position OrientationFeedForward::transformPosition()
{
    // return this->offset_rotation_.inverse()*this->current_ori_.inverse()*(d_pos_fixed_)-offset_position_;   
    // return (this->offset_.inverse()*this->current_.inverse()*this->desired_ee_).translation();
    return ((this->offset_.inverse()*this->current_.inverse()*this->desired_ee_)).translation();
}
OrientationFeedForward::Orientation OrientationFeedForward::transformOrientation()
{
    // return this->current_ori_.inverse()*this->offset_rotation_.inverse()*d_ori_fixed_;
    // return Eigen::Quaterniond((this->offset_.inverse()*this->current_.inverse()*this->desired_ee_).rotation());
    return Eigen::Quaterniond((this->offset_.inverse()*this->current_.inverse()*this->desired_ee_).rotation());
}
