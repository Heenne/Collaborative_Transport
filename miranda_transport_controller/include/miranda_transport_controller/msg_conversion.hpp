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


