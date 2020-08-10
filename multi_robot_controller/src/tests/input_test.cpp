#include<ros/ros.h>
#include<multi_robot_controller/input_pose_odom.hpp>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"input_test");
    ros::NodeHandle nh;
    // InputOdom base(nh,"/miranda/mir/odom_enc");
    InputPoseOdom base(nh,"/mur/mir/robot_pose_stamped","/mur/mir/odom_enc");

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