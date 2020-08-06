#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <multi_robot_controller/controller.h>


/**
 * @brief Class that implements a master robot for multi robot formation control
 * 
 */
class Master:public Controller{
    public:
        /**
         * @brief Construct a new Master object
         * 
         * @param nh Nodehandle
         */
        Master(std::string name,
                ros::NodeHandle nh=ros::NodeHandle("~"),
                ros::NodeHandle nh_topics=ros::NodeHandle("~"),
                ros::NodeHandle nh_parameters=ros::NodeHandle("~"));
    private:
};