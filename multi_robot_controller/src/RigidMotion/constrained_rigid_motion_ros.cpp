#include<multi_robot_controller/RigidMotion/constrained_rigid_motion_ros.h>

ConstrainedRigidMotionRos::ConstrainedRigidMotionRos():ConstrainedRigidMotion()
{
    ros::NodeHandle priv("~");
    std::vector<double> reference;
    if(priv.getParam("reference",reference))
    {
        if(reference.size()!=3)
        {
            std::invalid_argument("Reference vector has wrong number of elements!");
        }
        else
        {
            Eigen::Vector3d ref(reference.data());
            this->setReference(ref);
        }

    }
    else
    {
        throw NecessaryParamException(priv.resolveName("reference"));
    }
    
}