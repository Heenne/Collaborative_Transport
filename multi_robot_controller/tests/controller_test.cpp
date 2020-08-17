#include <multi_robot_controller/Controller/lyapunov_controller.h>


int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_controller");
    ros::NodeHandle nh;
    ros::NodeHandle priv("~");
    priv.setParam("rate",10);
    priv.setParam("topic_pose","pose");
    priv.setParam("topic_odom","odom");
    priv.setParam("topic_output","test_cmd_vel");
    priv.setParam("topic","topic");
    std::vector<float> lyap{7.0,7.0,11.0};
    priv.setParam("lyapunov",lyap);
    try{
        LyapunovController controller(nh);
        ros::spin();
    }
    catch(std::exception &ex)
    {
        ROS_ERROR_STREAM(ex.what());
    }  
       
    return 0;
}