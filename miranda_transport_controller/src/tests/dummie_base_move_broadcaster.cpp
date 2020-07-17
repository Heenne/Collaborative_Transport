#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>

#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(std_msgs::Float64 msg){
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg.data);
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
}

int main(int argc, char** argv){
  ros::init(argc, argv, "base_movement_dummie");
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.frame_id="world";
  transformStamped.child_frame_id="mir/base_link";
  br.sendTransform(transformStamped);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/test_float", 10, &poseCallback);

  ros::spin();
  return 0;
};