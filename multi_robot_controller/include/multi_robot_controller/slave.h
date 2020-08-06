#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <multi_robot_controller/controller.h>



#include <math.h>
#include <stdio.h>
#include <numeric>
/**
 * @brief Class that implements a slave robot for multi robot formation control
 * 
 */
class Slave:public Controller{
    public:
        /**
         * @brief Construct a new Slave object
         * 
         * @param nh NodeHandle
         */
        Slave(  std::string name,
                ros::NodeHandle nh=ros::NodeHandle("~"),
                ros::NodeHandle nh_topics=ros::NodeHandle("~"),
                ros::NodeHandle nh_parameters=ros::NodeHandle("~"));        
        void setMasterReference(tf::Pose pose);
    private:
        double old_angle_;

        ControlVector calcAngleDistance(AngleDistanceParameter parameter,ControlState target, ControlState current);
       
        ros::ServiceServer srv_set_reference_;     ///<Service for setting the reference pose
        tf::Transform master_reference_;
        /**
         * @brief Overloads the target odometry callback and expands it with transformation needed for slave robots.
         * 
         * @param msg  Incoming message at the target odometry topic
         */
        void targetOdomCallback(nav_msgs::Odometry msg);
        
        bool srvSetMasterReference(multi_robot_msgs::SetInitialPoseRequest &req,multi_robot_msgs::SetInitialPoseResponse &res );
};