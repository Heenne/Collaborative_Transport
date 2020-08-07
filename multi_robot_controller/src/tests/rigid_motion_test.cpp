#include <multi_robot_controller/constrained_rigid_motion.h>
#include <iostream>

int main(int argc,char** argv)
{
    ConstrainedRigidMotion motion(Eigen::Vector3d(0.0,1.0,0.0));
    double time=0.0;
    double omega=0.1;
    while(time<10.0)
    {
        Eigen::Vector3d pos(0.0,0.0,omega*time);
        Eigen::Vector3d vel(0.0,0.0,omega);
        motion.updateInputState(pos,vel,time);
        std::cout<<"State:"<<std::endl<<motion.getState()<<std::endl;
        std::cout<<"Diff"<<std::endl<<motion.getDiffState()<<std::endl;
        time+=omega;
    }
}