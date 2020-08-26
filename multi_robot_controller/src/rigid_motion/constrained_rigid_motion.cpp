#include<multi_robot_controller/rigid_motion/constrained_rigid_motion.h>
#include<iostream>

Constrainedrigid_motion::Constrainedrigid_motion()
{
    this->init();
}
Constrainedrigid_motion::Constrainedrigid_motion(Eigen::Vector3d reference)
{    
    this->init();
    this->setReference(reference);
   
}
void Constrainedrigid_motion::init()
{
    this->locking_=Constrainedrigid_motion::createDiffDriveLocking();
   
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
void Constrainedrigid_motion::setReference(Eigen::Vector3d ref)
{
    this->reference_=ref;
}

void Constrainedrigid_motion::updateInputState(Eigen::Vector3d state,Eigen::Vector3d d_state,double time)
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
Eigen::Vector3d Constrainedrigid_motion::getState()
{
    return this->state_out_;
}
Eigen::Vector3d Constrainedrigid_motion::getDiffState()
{
    return this->d_state_out_;
}

void Constrainedrigid_motion::calcConstrains()
{
    Eigen::Vector3d constrain_old=this->constrain_;
    double d_time=(this->time_new_-this->time_old_);

    if(!d_time==0.0 &&!this->initial_call_)
    {
        if(std::sqrt(std::pow(this->d_state_out_(0),2)+std::pow(this->d_state_out_(1),2))<0.05 )
        {
            
            this->constrain_<<0.0,0.0,this->state_in_(2)+this->reference_(2);
        }
        else
        {
            double angle_calc=std::atan2(this->d_state_out_(1),this->d_state_out_(0));
            this->constrain_<<0.0,0.0,angle_calc;            
        };
        // this->d_constrain_=(this->constrain_-constrain_old)/d_time;
        this->d_constrain_<<0.0,0.0,this->d_state_in_(2);
    }
    this->time_old_=this->time_new_;
}

void Constrainedrigid_motion::calcUnConstrained()
{
    //Rotate the reference position
    Eigen::Vector3d rot_ref=this->rotation_*Eigen::Vector3d(this->reference_(0),this->reference_(1),0.0);
    this->state_out_=this->state_in_+rot_ref; 
    //Add the reference angle
    this->state_out_(2)=this->state_in_(2)+this->reference_(2);
    //rotate the velocities
    this->d_state_out_=(this->d_state_in_+this->rotation_*this->angular_tensor_*this->reference_);
   
}

void Constrainedrigid_motion::applyConstrains()
{
    this->state_out_=this->locking_*this->state_out_+this->constrain_;
    this->d_state_out_=this->locking_*this->d_state_out_+this->d_constrain_;
    
}

Eigen::Matrix3d Constrainedrigid_motion::createDiffDriveLocking()
{
    Eigen::Matrix3d locking;
    locking=Eigen::Matrix3d::Identity();
    locking(2,2)=0.0;
    return locking;
}