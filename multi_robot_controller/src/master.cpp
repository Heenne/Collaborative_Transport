#include <multi_robot_controller/master.h>

Master::Master( std::string name,
                ros::NodeHandle nh,
                ros::NodeHandle nh_topics,
                ros::NodeHandle nh_parameters):Controller(name,nh,nh_topics,nh_parameters)
{
}
