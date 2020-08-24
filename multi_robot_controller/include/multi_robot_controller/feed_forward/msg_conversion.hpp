#ifndef MSG_CONVERSION_HPP
#define MSG_CONVERSION_HPP

#include<eigen3/Eigen/Dense>
#include<geometry_msgs/TransformStamped.h>
#include<geometry_msgs/Transform.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose.h>
#include<std_msgs/Float64.h>

/** @file */ 
/** @addtogroup multi_robot_controller
 * @{
 */

/**
 * @brief Convert an float64 z-axis angle to an eigen quaternion
 * 
 * @param msg Float64 ros message with z-axis angle
 * @param quat Eigen quaterniond the quaternion is stored in
 */
inline void convertMsg(Eigen::Quaterniond &quat,std_msgs::Float64 &msg)
{
   quat=    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(0.0,  Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(msg.data, Eigen::Vector3d::UnitZ());
}

/**
 * @brief Convert Orientation of a Transform geometry message to an eigen quaternion
 * 
 * @param msg Message that holds the transform
 * @param quat Quaternion the eigen quaternion is stored in
 */
inline void convertMsg(Eigen::Quaterniond &quat,geometry_msgs::Transform &msg)
{
    quat=Eigen::Quaterniond(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
}

/**
 * @brief Convert an StampedTransform geometry message to an eigen quaternion
 * 
 * @param msg Message that holds the stamped transform
 * @param quat Quternion the eigen quaternion is stored in
 */
inline void convertMsg(Eigen::Quaterniond &quat,geometry_msgs::TransformStamped &msg)
{
    convertMsg(quat,msg.transform);
}

/**
 * @brief Convert an Pose geometry message to an eigen quaternion
 * 
 * @param msg Message that holds the pose
 * @param quat Quternion the eigen quaternion is stored in
 */
inline void convertMsg(Eigen::Quaterniond &quat,geometry_msgs::Pose &msg)
{
    quat=Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
}
/**
 * @brief Convert an stamped pose geometry message to an eigen quaternion
 * 
 * @param msg Message that holds the stamped pose
 * @param quat Quternion the eigen quaternion is stored in
 */
inline void convertMsg(Eigen::Quaterniond &quat,geometry_msgs::PoseStamped &msg)
{
   convertMsg(quat,msg.pose);
}



/**
 * @brief Convert an pose geometry message to an Eigen pose vector (x, y, z, w, x, y, z)
 * 
 * @param msg Message that holds the pose
 * @param pose Matrix (Vector) the eigen pose vector is stored in
 */
inline void convertMsg(Eigen::Matrix<double,7,1> &pose,geometry_msgs::Pose &msg)
{
    pose<<  msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w;
   ;
}

/**
 * @brief Convert an stamepd pose geometry message to an Eigen pose vector (x, y, z, w, x, y, z)
 * 
 * @param msg Message that holds the pose
 * @param pose Matrix (Vector) the eigen pose vector is stored in
 */
inline void convertMsg(Eigen::Matrix<double,7,1> &pose,geometry_msgs::PoseStamped &msg)
{
    convertMsg(pose,msg.pose);
}

/**
 * @brief Convert  an Eigen pose vector (x, y, z, w, x, y, z) to a pose geometry message
 * 
 * @param msg Message that the pose is stored in
 * @param pose Matrix (Vector) that holds the pose
 */
inline void convertMsg(geometry_msgs::Pose &msg,Eigen::Matrix<double,7,1> &pose)
{
    msg.position.x=pose(0);
    msg.position.y=pose(1);
    msg.position.z=pose(2);
    msg.orientation.x=pose(3);
    msg.orientation.y=pose(4);
    msg.orientation.z=pose(5);
    msg.orientation.w=pose(6);
}

/**
 * @brief Convert  an Eigen pose vector (x, y, z, w, x, y, z) to a stamepd pose geometry message
 * 
 * @param msg Message that the pose is stored in
 * @param pose Matrix (Vector) that holds the pose
 */
inline void convertMsg(geometry_msgs::PoseStamped &msg,Eigen::Matrix<double,7,1> &pose)
{
    convertMsg(msg.pose,pose);
}

/**
 * @brief Convert  an Eigen pose vector (x, y, z, w, x, y, z) to a transform geometry message
 * 
 * @param msg Message that the pose is stored in
 * @param pose Matrix (Vector) that holds the pose
 */
inline void convertMsg(geometry_msgs::Pose &pose,geometry_msgs::Transform &transform)
{
    pose.position.x=transform.translation.x;
    pose.position.y=transform.translation.y;
    pose.position.z=transform.translation.z;

    pose.orientation.x=transform.rotation.x;
    pose.orientation.y=transform.rotation.y;
    pose.orientation.z=transform.rotation.z;
    pose.orientation.w=transform.rotation.w;
}

/**
 * @brief Convert a stamped transform geometry message to a stamped pose geometry message
 * 
 * @param pose Stamped pose the pose is stored in
 * @param transformStamped The geometry msgs that holds the transform
 */
inline void convertMsg(geometry_msgs::PoseStamped &pose,geometry_msgs::TransformStamped &transformStamped)
{
   pose.header=transformStamped.header;
   convertMsg(pose.pose,transformStamped.transform);
}

/**
 * @brief Convert a stamped transform geometry message to a eigen pose vector
 * 
 * @param pose Pose Vector the pose is stored in
 * @param transformStamped The geometry msgs that holds the transform
 */
inline void convertMsg(Eigen::Matrix<double,7,1> &pose,geometry_msgs::TransformStamped &transformStamped)
{
    geometry_msgs::PoseStamped pose_msg;
    convertMsg(pose_msg,transformStamped);
    convertMsg(pose,pose_msg);
}

/**
 * @brief Convert a geometry pos messag to a geometry transfomr message
 * 
 * @param transform Message the transform is stored in
 * @param pose Message that holds the pose
 */
inline void convertMsg(geometry_msgs::Transform &transform,geometry_msgs::Pose &pose)
{
    transform.translation.x=pose.position.x;
    transform.translation.y=pose.position.y;
    transform.translation.z=pose.position.z;

    transform.rotation.x=pose.orientation.x;
    transform.rotation.y=pose.orientation.y;
    transform.rotation.z=pose.orientation.z;
    transform.rotation.w=pose.orientation.w;
    
    
}

/**
 * @brief Convert a geometry stamped pose message to a stamped transform geometry message
 * 
 * @param transform Message the stamped transform is stored in
 * @param pose Message that holds the pose
 */
inline void convertMsg(geometry_msgs::TransformStamped &transform,geometry_msgs::PoseStamped &pose)
{
    transform.header=pose.header;
    convertMsg(transform.transform,pose.pose);
}

#endif