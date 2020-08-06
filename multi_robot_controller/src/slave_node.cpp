#include <multi_robot_controller/slave.h>

int main (int argc,char** argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle node_;
    ros::NodeHandle parameter_ns(node_.resolveName("controller"));

   
    Slave slave=Slave(argv[1],node_,node_,parameter_ns);
    ros::spin();
}