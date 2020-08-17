#include<ros/ros.h>
#include<multi_robot_controller/Input/input_pose_odom.hpp>
#include<multi_robot_controller/Input/input_odom.hpp>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"input_test");
    ros::NodeHandle nh("/mur/mir");
    
    // InputPoseOdom base(nh,"robot_pose_stamped","odom_enc");
    // InputOdom base(nh,"/miranda/mir/odom_enc");

    ros::NodeHandle priv("~");
    priv.setParam("topic_pose","robot_pose_stamped");
    priv.setParam("topic_odom","odom_enc");
    try{
            InputPoseOdom base(nh);
            // InputOdom base(nh);
            ros::Rate rate(1);
            while (ros::ok())
            {
                tf::Pose pose=base.getPose();
                tf::Vector3 lin=base.getLinVel();
                tf::Vector3 ang=base.getAngVel();
                std::cout<<"Pose:"<<std::endl<<
                                    pose.getOrigin().x()<<
                                    "\t"<<pose.getOrigin().y()<<
                                    "\t"<<pose.getOrigin().z()<<
                                    "\n"<<pose.getRotation().x()<<
                                    "\t"<<pose.getRotation().y()<<
                                    "\t"<<pose.getRotation().z()<<
                                    "\t"<<pose.getRotation().w()<<"\n";

                std::cout<<"Velocity:"<<std::endl<<
                                lin.x()<<
                                    "\t"<<lin.y()<<
                                    "\t"<<lin.z()<<
                                    "\n"<<ang.x()<<
                                    "\t"<<ang.y()<<
                                    "\t"<<ang.z()<<"\n";

                    
                rate.sleep();
                ros::spinOnce();
            }   
    }
    catch(std::exception &ex)
    {
        ROS_WARN_STREAM(ex.what());
    }
   
   
}