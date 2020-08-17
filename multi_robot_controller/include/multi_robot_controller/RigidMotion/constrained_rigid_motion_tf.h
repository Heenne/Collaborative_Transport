#include<multi_robot_controller/RigidMotion/constrained_rigid_motion_ros.h>
#include<ros/ros.h>
#include<tf/tf.h>

class ConstrainedRigidMotionTf:public ConstrainedRigidMotionRos{
    public:
        ConstrainedRigidMotionTf();
        void updateInputState(tf::Pose pose,tf::Vector3 lin_vel, tf::Vector3 ang_vel,double time);
        using ConstrainedRigidMotion::updateInputState;
    private:
};