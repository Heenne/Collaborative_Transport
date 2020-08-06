#include <multi_robot_controller/master.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle node_;
    ros::NodeHandle robot_ns_("/"+std::string(argv[1]));
    ros::NodeHandle parameter_ns(robot_ns_.resolveName("controller")); 
   
    Master master=Master(argv[1],node_,robot_ns_,parameter_ns);
    ros::spin();
}