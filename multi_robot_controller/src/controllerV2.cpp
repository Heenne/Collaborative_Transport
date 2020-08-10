#include<multi_robot_controller/controllerV2.h>
Controller::Controller()
{

}

Controller::Controller(ros::NodeHandle &nh)
{
    this->nh_=nh;
    switch(InputTypes::POSE_ODOM)
    {
        case InputTypes::POSE_ODOM:
        {
            this->current_state_handler_=new InputPoseOdom(this->nh_);         
            this->target_state_handler_=new InputPoseOdom(this->nh_);
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
    //Get the untransformed target state (ledader state)
    State untransformed(    this->target_state_handler_->getPose(),
                            this->target_state_handler_->getLinVel(),
                            this->target_state_handler_->getAngVel());

    //convert input states to eigen state vectors for the rigid motion
    //For state
    Eigen::Vector3d state_rigid(this->target_state_handler_->getPose().getOrigin().x(),
                                this->target_state_handler_->getPose().getOrigin().y(),
                                tf::getYaw(this->target_state_handler_->getPose().getRotation()));
    //For state derivative                        
    Eigen::Vector3d d_state_rigid(  this->target_state_handler_->getLinVel().x(),
                                    this->target_state_handler_->getLinVel().y(),
                                    this->target_state_handler_->getLinVel().z());
    
    //Update the state of rigid motion
    this->rigid_motion_.updateInputState(state_rigid,d_state_rigid,this->target_state_handler_->getTime().toSec());
    Eigen::Vector3d state_vector=rigid_motion_.getState();
    Eigen::Vector3d d_state_vector=rigid_motion_.getDiffState();
    
    //Rewrite the states into the state definitions of the controller
    this->target_state_=State(tf::Pose(tf::createQuaternionFromYaw(state_vector(2)),tf::Vector3(state_vector(0),state_vector(1),0.0)),
                              tf::Vector3(d_state_vector(0),d_state_vector(1),0.0),
                              tf::Vector3(0.0,0.0,d_state_vector(2)));

    //Execute the calculations from derived classes
    this->control_=this->calcControl(this->current_state_,this->target_state_);

    this->publish();
}

void Controller::publish()
{
    geometry_msgs::Twist twist;
    twist.linear.x=this->control_.v;
    twist.angular.z=this->control_.omega;
    this->pub_.publish(twist);
}