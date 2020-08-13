#include<multi_robot_controller/constrained_rigid_motion.h>
#include<iostream>

ConstrainedRigidMotion::ConstrainedRigidMotion()
{
    this->init();
}
ConstrainedRigidMotion::ConstrainedRigidMotion(Eigen::Vector3d reference)
{    
    this->init();
    this->setReference(reference);
   
}
void ConstrainedRigidMotion::init()
{
    this->locking_=ConstrainedRigidMotion::createDiffDriveLocking();
   
    this->state_in_=Eigen::Vector3d::Zero();
    this->d_state_in_=Eigen::Vector3d::Zero();
    
    this->angular_tensor_=Eigen::Matrix3d::Zero();
    this->rotation_=Eigen::Quaterniond::Identity();
    this->constrain_=Eigen::Vector3d::Zero();
    this->d_constrain_=Eigen::Vector3d::Zero();
    
    this->state_out_=this->state_in_;
    this->d_state_out_=this->d_state_in_;
    
    this->time_old_=0.0;
    this->time_new_=this->time_old_;
    this->initial_call_=true;  
}
void ConstrainedRigidMotion::setReference(Eigen::Vector3d ref)
{
    this->reference_=ref;
}

void ConstrainedRigidMotion::updateInputState(Eigen::Vector3d state,Eigen::Vector3d d_state,double time)
{

    this->time_new_=time;
    this->state_in_=state;
    this->d_state_in_=d_state;
    this->rotation_=Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ());
    
    this->angular_tensor_(0,1)=-this->d_state_in_(2);
    this->angular_tensor_(1,0)=this->d_state_in_(2);
    
    calcUnConstrained();
    calcConstrains();
    applyConstrains();
    this->initial_call_=false;
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
    if(!d_time==0.0 &&!this->initial_call_)
    {
        if(this->d_state_out_(0)!=0.0)
        {
            this->constrain_<<0.0,0.0,std::atan2(this->d_state_out_(1),this->d_state_out_(0));
        }
        else
        {
            this->constrain_<<0.0,0.0,0.0;
        }
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

Eigen::Matrix3d ConstrainedRigidMotion::createDiffDriveLocking()
{
    Eigen::Matrix3d locking;
    locking=Eigen::Matrix3d::Identity();
    locking(2,2)=0.0;
    return locking;
}