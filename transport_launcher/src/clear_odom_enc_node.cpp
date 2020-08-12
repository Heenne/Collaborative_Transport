#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include <std_srvs/Empty.h>
#include <tf/tf.h>

bool reset;
geometry_msgs::TransformStamped trafo_stamped;
tf2_ros::Buffer tfBuffer;

void callbackRobotPose(nav_msgs::Odometry msg)
{
    
   
    if(reset)
    {
        
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        tf2::Transform trafo1;
        try{
            transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
                                ros::Time(0));
       
            tf2::fromMsg(transformStamped.transform,trafo1);
            
            tf2::Transform trafo2;
            tf2::fromMsg(msg.pose.pose,trafo2);
            trafo2=trafo2.inverse(); 
            trafo_stamped.transform=tf2::toMsg(trafo1*trafo2);  
            trafo_stamped.header.frame_id="odom";
            trafo_stamped.child_frame_id="odom_enc";
            trafo_stamped.header.stamp=msg.header.stamp;
            reset=false;    
            static tf2_ros::StaticTransformBroadcaster static_broadcaster;
            static_broadcaster.sendTransform(trafo_stamped);   
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
      
    }
  

    
}

bool srvReset(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{
    reset=true;
    return true;
}

int main(int argc, char** argv)
{
    reset=false;
   

    ros::init(argc,argv,"clear_odom_enc");
    ros::NodeHandle nh; 
    trafo_stamped.header.frame_id="odom";
    trafo_stamped.child_frame_id="odom_enc";
    trafo_stamped.header.stamp=ros::Time::now();
    trafo_stamped.transform=tf2::toMsg(tf2::Transform::getIdentity());
    ros::Subscriber sub=nh.subscribe("odom_enc",10,callbackRobotPose);
    ros::ServiceServer service = nh.advertiseService("reset",srvReset);

    ros::spin();

}