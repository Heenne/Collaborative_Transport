#include <multi_robot_controller/lyapunov_controller.hpp>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    LyapunovController master(nh);
    ros::spin();
}