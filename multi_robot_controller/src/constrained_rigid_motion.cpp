#include<multi_robot_controller/constrained_rigid_motion.h>

ConstrainedRigidMotion::ConstrainedRigidMotion(Eigen::Vector3d reference)
{
   this->reference_=reference;
   this->locking_=Eigen::Matrix3d::Identity();
   this->angular_tensor_=Eigen::Matrix3d::Zero();
   this->locking_(3,3)=0.0;
   

   
}

void ConstrainedRigidMotion::updateInputState(Eigen::Vector3d state,Eigen::Vector3d d_state,double time)
{
    this->time_new_=time;
    this->state_in_=state;
    this->d_state_in_=d_state;
    this->rotation_=Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitZ());
    this->angular_tensor_(1,2)=-this->d_state_in_(3);
    this->angular_tensor_(2,1)=this->d_state_in_(3);
    
    calcUnConstrained();
    calcConstrains();
    applyConstrains();
}
Eigen::Vector3d ConstrainedRigidMotion::getState()
{
    return this->state_out_;
}
Eigen::Vector3d ConstrainedRigidMotion::getDiffState()
{
    return this->d_state_out_;
}

void ConstrainedRigidMotion::calcConstrains()
{
    Eigen::Vector3d constrain_old=this->constrain_;
    double d_time=(this->time_new_-this->time_old_);
    if(!d_time==0.0)
    {
        this->constrain_=Eigen::Vector3d (0.0,0.0,std::atan2(this->d_state_out_(1),this->d_state_out_(3)));
        this->d_constrain_=(this->constrain_-constrain_old)/d_time;
    }
    this->time_old_=this->time_new_;
}
void ConstrainedRigidMotion::calcUnConstrained()
{
    this->state_out_=(this->state_in_+this->rotation_*this->reference_);
    this->d_state_out_=(this->d_state_in_+this->angular_tensor_*this->rotation_*this->reference_);
}
void ConstrainedRigidMotion::applyConstrains()
{
    this->d_state_out_=this->locking_*this->d_state_out_+this->d_constrain_;
    this->state_out_=this->locking_*this->state_out_+this->constrain_;
}