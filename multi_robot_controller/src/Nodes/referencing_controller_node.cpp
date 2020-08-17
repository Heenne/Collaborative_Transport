#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_srvs/Empty.h>

geometry_msgs::PoseStamped pose;

void callbackPose(geometry_msgs::PoseStamped &msg)
{
    pose=msg;
}

bool callbackReferenceSrv(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{

    return true;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"referencing_ode");
    ros::NodeHandle nh;
    ros::Subscriber sub=nh.subscribe("topic_in",10,callbackPose);
    ros::NodeHandle priv("~");
    ros::ServiceServer srv=priv.advertiseService<std_srvs::Empty>("reference",callbackReferenceSrv);
}