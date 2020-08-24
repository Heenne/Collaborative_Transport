#include <multi_robot_controller/controller/lyapunov_controller.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"master");
    ros::NodeHandle nh;
    ros::NodeHandle priv("~");
    priv.setParam("reference",std::vector<double>{0.0,0.0,0.0});
    LyapunovController master(nh);
    ros::spin();
}