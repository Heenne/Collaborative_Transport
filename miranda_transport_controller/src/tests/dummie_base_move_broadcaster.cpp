#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

#include <turtlesim/Pose.h>

float angle=0.0;
void poseCallback(std_msgs::Float64 msg){
  angle=msg.data;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_movement_dummie");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/test_float", 10, &poseCallback);
  static tf2_ros::TransformBroadcaster br;
  ros::Rate rate(500);

  geometry_msgs::TransformStamped transformStamped;
  while(ros::ok())
  {
    tf2::Quaternion q;
    q.setRPY(0, 0, angle);
    transformStamped.header.frame_id="world";
    transformStamped.header.stamp=ros::Time::now();
    transformStamped.child_frame_id="base_link";
    transformStamped.transform.translation.x=0.0;
    transformStamped.transform.translation.y=0.0;
    transformStamped.transform.translation.z=0.0;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    br.sendTransform(transformStamped);
    rate.sleep();
    ros::spinOnce();
  } 
  return 0;
};