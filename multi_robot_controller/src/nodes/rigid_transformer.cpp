#include<ros/ros.h>
#include<multi_robot_controller/input/input_base.hpp>
#include<multi_robot_controller/input/input_pose_odom.hpp>
#include<multi_robot_controller/input/input_types.hpp>
#include<multi_robot_controller/rigid_motion/constrained_rigid_motion_tf.h>
#include<geometry_msgs/PoseStamped.h>
int main(int argc, char** argv)
{
    ros::init(argc,argv,"rigid_transformer");
    ros::NodeHandle nh;
    ros::NodeHandle priv("~");
    ros::Publisher pub=nh.advertise<geometry_msgs::PoseStamped>("rigid_transformed",10);
    int input_type;
    priv.getParam("input_type",input_type);
    InputBase* input;
    switch(InputTypes(input_type))
    {
        case InputTypes::POSE_ODOM:
            input = new InputPoseOdom(nh,priv);
            break;
        default:
            break;
    }
    ros::Rate rate(10);
    while (ros::ok())
    {
        ConstrainedRigidMotionTf rigid;
        rigid.updateInputState(input->getPose(),input->getLinVel(),input->getAngVel(),input->getTime().toSec());
         
        geometry_msgs::PoseStamped pose_transformed;
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(rigid.getState()(2)),tf::Vector3(rigid.getState()(0),rigid.getState()(1),0.0)),pose_transformed.pose);
        pub.publish(pose_transformed);
        ros::spinOnce();
        rate.sleep();       
    }
    

}