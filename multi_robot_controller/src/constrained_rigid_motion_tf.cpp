#include<multi_robot_controller/constrained_rigid_motion_tf.h>

ConstrainedRigidMotionTf::ConstrainedRigidMotionTf():ConstrainedRigidMotionRos()
{

}

void ConstrainedRigidMotionTf::updateInputState(tf::Pose pose,tf::Vector3 lin_vel, tf::Vector3 ang_vel,double time)
{
    //convert input states to eigen state vectors for the rigid motion
    //For state  

    Eigen::Vector3d state_rigid;
    state_rigid<<   pose.getOrigin().x(),
                    pose.getOrigin().y(),
                    tf::getYaw(pose.getRotation());
   
    //For state derivative                        
    Eigen::Vector3d d_state_rigid;
    d_state_rigid<< lin_vel.x(),
                    lin_vel.y(),
                    ang_vel.z();
                                    
    this->updateInputState(state_rigid,d_state_rigid,time);            

}