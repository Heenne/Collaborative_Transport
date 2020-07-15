#ifndef MSG_CONVERSION_HPP
#define MSG_CONVERSION_HPP
#include<eigen3/Eigen/Dense>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Transform.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<std_msgs/Float64.h>

void convertMsg(std_msgs::Float64 &msg,Eigen::Quaterniond &quat)
{
   quat=    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(msg.data, Eigen::Vector3d::UnitZ());
}

void convertMsg(geometry_msgs::Transform &msg,Eigen::Quaterniond &quat)
{
    Eigen::Vector4d vec;
    vec<<msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z;
    quat=Eigen::Quaterniond(vec);
}
void convertMsg(geometry_msgs::TransformStamped &msg,Eigen::Quaterniond &quat)
{
    convertMsg(msg.transform,quat);
}

void convertMsg(geometry_msgs::Pose &msg,Eigen::Quaterniond &quat)
{
    Eigen::Vector4d vec;
    vec<<msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z;
    quat=Eigen::Quaterniond(vec);
}

void convertMsg(geometry_msgs::PoseStamped &msg,Eigen::Quaterniond &quat)
{
   convertMsg(msg.pose,quat);
}




void convertMsg(Eigen::Matrix<double,7,1> &pose,geometry_msgs::Pose &msg)
{
    pose<<  msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z;
   ;
}

void convertMsg(Eigen::Matrix<double,7,1> &pose,geometry_msgs::PoseStamped &msg)
{
    convertMsg(pose,msg.pose);
}

void convertMsg(geometry_msgs::Pose &msg,Eigen::Matrix<double,7,1> &pose)
{
    msg.position.x=pose(0);
    msg.position.y=pose(1);
    msg.position.z=pose(2);
    msg.orientation.w=pose(3);
    msg.orientation.x=pose(4);
    msg.orientation.y=pose(5);
    msg.orientation.z=pose(6);
}

void convertMsg(geometry_msgs::PoseStamped &msg,Eigen::Matrix<double,7,1> &pose)
{
    convertMsg(msg.pose,pose);
}

void convertMsg(geometry_msgs::Pose &pose,geometry_msgs::Transform &transform)
{
    pose.position.x=transform.translation.x;
    pose.position.y=transform.translation.y;
    pose.position.z=transform.translation.z;

    pose.orientation.x=transform.rotation.x;
    pose.orientation.y=transform.rotation.y;
    pose.orientation.z=transform.rotation.z;
    pose.orientation.w=transform.rotation.w;
}
void convertMsg(geometry_msgs::PoseStamped &pose,geometry_msgs::TransformStamped &transformStamped)
{
   pose.header=transformStamped.header;
   convertMsg(pose.pose,transformStamped.transform);
}

void convertMsg(geometry_msgs::Transform &transform,geometry_msgs::Pose &pose)
{
    transform.translation.x=pose.position.x;
    transform.translation.y=pose.position.y;
    transform.translation.z=pose.position.z;

    transform.rotation.x=pose.orientation.x;
    transform.rotation.y=pose.orientation.y;
    transform.rotation.z=pose.orientation.z;
    transform.rotation.w=pose.orientation.w;
    
    
}

void convertMsg(geometry_msgs::TransformStamped &transform,geometry_msgs::PoseStamped &pose)
{
    transform.header=pose.header;
    convertMsg(transform,pose);
}

#endif