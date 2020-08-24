#include<multi_robot_controller/rigid_motion/constrained_rigid_motion_ros.h>
#include<ros/ros.h>
#include<tf/tf.h>

class Constrainedrigid_motionTf:public Constrainedrigid_motionRos{
    public:
        Constrainedrigid_motionTf();
        void updateInputState(tf::Pose pose,tf::Vector3 lin_vel, tf::Vector3 ang_vel,double time);
        using Constrainedrigid_motion::updateInputState;
    private:
};