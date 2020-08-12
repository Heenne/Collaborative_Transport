#include <multi_robot_controller/lyapunov_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"slave");
    ros::NodeHandle nh;
    LyapunovController slave(nh);
    ros::spin();
}