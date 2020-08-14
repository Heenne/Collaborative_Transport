#include<multi_robot_controller/controller.h>
Controller::State::State(tf::Pose pose,tf::Vector3 lin_vel,tf::Vector3 ang_vel)
{
    this->pose=pose;
    this->lin_vel=lin_vel;
    this->ang_vel=ang_vel;
}
Controller::State::State()
{
    this->pose=tf::Pose(tf::createIdentityQuaternion(),tf::Vector3(0.0,0.0,0.0));
    this->lin_vel=tf::Vector3(0.0,0.0,0.0);
    this->ang_vel=tf::Vector3(0.0,0.0,0.0);
}





Controller::Controller()
{

}

Controller::Controller(ros::NodeHandle &nh)
{
    this->nh_=nh;
    this->enable_=false;

    ros::NodeHandle current_param("~/current");
    ros::NodeHandle target_param("~/target");

    switch(InputTypes::POSE_ODOM)   //TODO: type sensitivity
    {
        case InputTypes::POSE_ODOM:
        {
            ROS_INFO("Binding input and output to Pose+Odometry topics!");
            this->current_state_handler_=new InputPoseOdom(this->nh_,current_param);         
            this->target_state_handler_=new InputPoseOdom(this->nh_,target_param);
            break;
        }
        case InputTypes::POSE_TWIST:
        {
            this->current_state_handler_=new InputPoseTwist(this->nh_);         
            this->target_state_handler_=new InputPoseTwist(this->nh_);
            break;
        }
        case InputTypes::SINGLE_ODOM:
        {
            this->current_state_handler_=new InputOdom(this->nh_);         
            this->target_state_handler_=new InputOdom(this->nh_);
            break;
        }
    }
   


    ros::NodeHandle priv("~");

    float rate;
    if(priv.getParam("rate",rate))
    {
        ROS_INFO("Setting control rate %lf",rate);
        this->control_scope_timer_=this->nh_.createTimer(ros::Duration(1.0/rate),&Controller::controlScope,this);
    } 
    else
    {
        throw NecessaryParamException(priv.resolveName("rate"));
    }


    std::string output_topic;
    if(priv.getParam("topic_output",output_topic))
    {
        this->pub_=this->nh_.advertise<geometry_msgs::Twist>(output_topic,10);
    }
    else
    {
        throw NecessaryParamException(priv.resolveName("topic_output"));
    }

    this->meta_=priv.advertise<multi_robot_msgs::MetaData>("meta_data",10); 
    this->enable_srv_=priv.advertiseService("enable_controller",&Controller::enableCallback,this);
    this->disable__srv=priv.advertiseService("disable_controller",&Controller::disableCallback,this);
    
}

Controller::~Controller()
{
    delete this->current_state_handler_;
    delete this->target_state_handler_;
}

void Controller::controlScope(const ros::TimerEvent&)
{
    //Get the current state from handler
    this->current_state_=State( this->current_state_handler_->getPose(),
                                this->current_state_handler_->getLinVel(),
                                this->current_state_handler_->getAngVel());

    //Get the untransformed target state (leader state)
    State leader(   this->target_state_handler_->getPose(),
                    this->target_state_handler_->getLinVel(),
                    this->target_state_handler_->getAngVel());
        
        
    //Update the state of rigid motion
    this->rigid_motion_.updateInputState(   leader.pose,
                                            leader.lin_vel,
                                            leader.ang_vel,
                                            this->target_state_handler_->getTime().toSec());

    Eigen::Vector3d state_vector=rigid_motion_.getState();
    Eigen::Vector3d d_state_vector=rigid_motion_.getDiffState();
    
    //Rewrite the states into the state definitions of the controller
    this->target_state_=State(tf::Pose(tf::createQuaternionFromYaw(state_vector(2)),tf::Vector3(state_vector(0),state_vector(1),0.0)),
                              tf::Vector3(d_state_vector(0),d_state_vector(1),0.0),
                              tf::Vector3(0.0,0.0,d_state_vector(2)));

    //Execute the calculations from derived classes   
    this->publish();
}

void Controller::publish()
{
    //Publish control command
    if(this->enable_)
    {
        geometry_msgs::Twist twist;
        twist.linear.x=this->control_.v;
        twist.angular.z=this->control_.omega;
        this->pub_.publish(twist);
    }
    

    //publish Metadata    
    this->publishMetaData();

}

void Controller::publishMetaData()
{
    multi_robot_msgs::State current_state_msg;
    tf::poseTFToMsg(this->current_state_.pose,current_state_msg.pose.pose);
    current_state_msg.pose.header.stamp=this->current_state_handler_->getTime();
    tf::vector3TFToMsg(this->current_state_.lin_vel,current_state_msg.lin_vel);
    tf::vector3TFToMsg(this->current_state_.ang_vel,current_state_msg.ang_vel);

    multi_robot_msgs::State target_state_msg;
    tf::poseTFToMsg(this->target_state_.pose,target_state_msg.pose.pose);
    target_state_msg.pose.header.stamp=this->target_state_handler_->getTime();
    tf::vector3TFToMsg(this->target_state_.lin_vel,target_state_msg.lin_vel);
    tf::vector3TFToMsg(this->target_state_.ang_vel,target_state_msg.ang_vel);

    multi_robot_msgs::State diff_msg;
    State diff_state(   this->target_state_.pose.inverseTimes(this->current_state_.pose),
                        this->target_state_.lin_vel-this->current_state_.lin_vel,
                        this->target_state_.ang_vel-this->current_state_.ang_vel);
    
    tf::poseTFToMsg(diff_state.pose,target_state_msg.pose.pose);
    // target_state_msg.pose.header.stamp=ros::Time((this->target_state_handler_->getTime()-this->current_state_handler_->getTime()).toSec());
    tf::vector3TFToMsg(diff_state.lin_vel,diff_msg.lin_vel);
    tf::vector3TFToMsg(diff_state.ang_vel,diff_msg.ang_vel);


    multi_robot_msgs::MetaData msg;
    msg.current=current_state_msg;
    msg.target=target_state_msg;
    msg.diff=diff_msg;


    this->meta_.publish(msg);
}

bool Controller::enableCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyRequest &res)
{
    this->enable_=true;
}
bool Controller::disableCallback(std_srvs::EmptyRequest &req,std_srvs::EmptyRequest &res)
{

    this->enable_=false;
}
        